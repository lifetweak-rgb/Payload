#include "cattle/cattle_payload.hpp"

#include "cattle_config.hpp"
#include "cattle_counter.hpp"
#include "cattle_report.hpp"

extern "C" {
#include <dji_aircraft_info.h>
#include <dji_fc_subscription.h>
#include <dji_high_speed_data_channel.h>
#include <dji_liveview.h>
#include <dji_logger.h>
#include <dji_payload_camera.h>
#include <dji_platform.h>
}

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <time.h>
#include <vector>

#ifdef OPEN_CV_INSTALLED
#include <opencv2/opencv.hpp>
#include "liveview/image_processor_yolovfastest.hpp"
#endif

namespace {

const double kPi = 3.14159265358979323846;

struct SharedState {
    std::atomic<bool> running;
    CattleAppConfig config;
    CattleCounter *counter;
    T_DjiMutexHandle image_mutex;
    T_DjiMutexHandle counter_mutex;
    T_DjiMutexHandle meta_mutex;
    double ground_altitude_m;
    bool ground_set;
    bool logged_coco_score;
    int frames_seen;
    FrameResult last_result;
    CameraPose last_pose;
    bool have_pose;
    std::string aircraft_name;
    std::string latest_image;
#ifdef OPEN_CV_INSTALLED
    std::queue<cv::Mat> images;
    ImageProcessorYolovFastest *yolo;
    cv::dnn::Net onnx;
    bool use_onnx;
#endif
    std::queue<T_DjiLiveViewStandardMetaData *> metas;

    SharedState()
        : running(false),
          counter(0),
          image_mutex(0),
          counter_mutex(0),
          meta_mutex(0),
          ground_altitude_m(0.0),
          ground_set(false),
          logged_coco_score(false),
          frames_seen(0),
          have_pose(false),
          aircraft_name("DJI aircraft"),
#ifdef OPEN_CV_INSTALLED
          yolo(0),
          use_onnx(false),
#endif
          metas() {
        last_result.in_frame = 0;
        last_result.confirmed = 0;
        last_result.candidates = 0;
        last_result.used_gps = false;
        last_result.skip = CATTLE_SKIP_NONE;
    }
};

SharedState *g_state = 0;

bool EnsureDirectory(const std::string &path) {
    if (path.empty()) {
        return false;
    }
    std::string current;
    for (size_t i = 0; i < path.size(); ++i) {
        current.push_back(path[i]);
        if (path[i] == '/' && current.size() > 1) {
            mkdir(current.c_str(), 0755);
        }
    }
    if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) {
        return false;
    }
    return true;
}

std::string UtcNow() {
    std::time_t now = std::time(0);
    std::tm utc;
    gmtime_r(&now, &utc);
    char buffer[40];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S UTC", &utc);
    return buffer;
}

bool CopyFile(const std::string &from, const std::string &to) {
    std::ifstream input(from.c_str(), std::ios::binary);
    if (!input) {
        return false;
    }
    std::ofstream output(to.c_str(), std::ios::binary);
    if (!output) {
        return false;
    }
    output << input.rdbuf();
    return output.good();
}

bool ReadFusedAltitude(double *altitude_m) {
    T_DjiFcSubscriptionAltitudeFused altitude = 0.0f;
    T_DjiReturnCode code = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, reinterpret_cast<uint8_t *>(&altitude),
        sizeof(altitude), 0);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return false;
    }
    *altitude_m = altitude;
    return true;
}

bool ReadPose(const CattleAppConfig &config, int width, int height, CameraPose *pose) {
    pose->time_s = 0.0;
    pose->lat_deg = 0.0;
    pose->lon_deg = 0.0;
    pose->alt_agl_m = 0.0;
    pose->gimbal_pitch_deg = 0.0;
    pose->gimbal_yaw_deg = 0.0;
    pose->width = width;
    pose->height = height;
    pose->gps_valid = false;

    struct timespec clock_time;
    if (clock_gettime(CLOCK_MONOTONIC, &clock_time) == 0) {
        pose->time_s = static_cast<double>(clock_time.tv_sec) + static_cast<double>(clock_time.tv_nsec) * 1e-9;
    }

    T_DjiFcSubscriptionPositionFused position = {};
    T_DjiFcSubscriptionGimbalAngles gimbal = {};
    T_DjiReturnCode position_code = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, reinterpret_cast<uint8_t *>(&position), sizeof(position), 0);
    T_DjiReturnCode gimbal_code = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, reinterpret_cast<uint8_t *>(&gimbal), sizeof(gimbal), 0);
    double altitude_m = 0.0;
    bool have_altitude = ReadFusedAltitude(&altitude_m);
    if (gimbal_code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        pose->gimbal_pitch_deg = gimbal.x;
        pose->gimbal_yaw_deg = gimbal.z;
    }
    if (position_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS || !have_altitude || !g_state->ground_set) {
        return position_code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    double lat = position.latitude;
    double lon = position.longitude;
    if (std::fabs(lat) <= kPi + 0.01) {
        lat = lat * 180.0 / kPi;
    }
    if (std::fabs(lon) <= kPi + 0.01) {
        lon = lon * 180.0 / kPi;
    }
    pose->lat_deg = lat;
    pose->lon_deg = lon;
    pose->alt_agl_m = altitude_m - g_state->ground_altitude_m;
    pose->gps_valid = position.visibleSatelliteNumber >= 6;
    (void)config;
    return true;
}

void PushMeta(T_DjiLiveViewStandardMetaData *meta) {
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    osal->MutexLock(g_state->meta_mutex);
    while (g_state->metas.size() > 2) {
        T_DjiLiveViewStandardMetaData *old = g_state->metas.front();
        g_state->metas.pop();
        free(old);
    }
    g_state->metas.push(meta);
    osal->MutexUnlock(g_state->meta_mutex);
}

T_DjiLiveViewStandardMetaData *PopMeta() {
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    T_DjiLiveViewStandardMetaData *meta = 0;
    osal->MutexLock(g_state->meta_mutex);
    if (!g_state->metas.empty()) {
        meta = g_state->metas.front();
        g_state->metas.pop();
    }
    osal->MutexUnlock(g_state->meta_mutex);
    return meta;
}

void FillMeta(const FrameResult &result, int width, int height, T_DjiLiveViewStandardMetaData **out) {
    size_t count = result.hits.size();
    size_t bytes = sizeof(T_DjiLiveViewStandardMetaData) + count * sizeof(T_DjiLiveViewBoundingBox);
    T_DjiLiveViewStandardMetaData *meta = static_cast<T_DjiLiveViewStandardMetaData *>(malloc(bytes));
    if (!meta) {
        *out = 0;
        return;
    }
    meta->boxCount = static_cast<uint8_t>(count > 255 ? 255 : count);
    for (size_t i = 0; i < meta->boxCount; ++i) {
        const FrameHit &hit = result.hits[i];
        meta->boxData[i].id = static_cast<uint16_t>(hit.id);
        meta->boxData[i].type = 0;
        meta->boxData[i].state = hit.confirmed ? DJI_LIVEVIEW_OBJ_STATE_TRACKED
                                               : DJI_LIVEVIEW_OBJ_STATE_NOT_CONFIDENT;
        float cx = hit.cx * 10000.0f / static_cast<float>(width);
        float cy = hit.cy * 10000.0f / static_cast<float>(height);
        float w = hit.w * 10000.0f / static_cast<float>(width);
        float h = hit.h * 10000.0f / static_cast<float>(height);
        if (cx < 0) cx = 0;
        if (cy < 0) cy = 0;
        if (w < 0) w = 0;
        if (h < 0) h = 0;
        if (cx > 10000) cx = 10000;
        if (cy > 10000) cy = 10000;
        if (w > 10000) w = 10000;
        if (h > 10000) h = 10000;
        meta->boxData[i].box.cx = static_cast<uint16_t>(cx);
        meta->boxData[i].box.cy = static_cast<uint16_t>(cy);
        meta->boxData[i].box.w = static_cast<uint16_t>(w);
        meta->boxData[i].box.h = static_cast<uint16_t>(h);
        meta->boxData[i].box.distance = 0;
    }
    *out = meta;
}

int EmailCount(const char *reason) {
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    osal->MutexLock(g_state->counter_mutex);
    std::string report = BuildCattleReport(*g_state->counter, g_state->aircraft_name, UtcNow());
    int headline = CattleHeadlineCount(*g_state->counter);
    osal->MutexUnlock(g_state->counter_mutex);

    EnsureDirectory(g_state->config.output_dir);
    std::string body_path = g_state->config.output_dir + "/cattle_report.txt";
    std::string error;
    if (!WriteTextFile(body_path, report, &error)) {
        USER_LOG_ERROR("Could not write the cattle report: %s", error.c_str());
        return -1;
    }
    std::string attachment;
    std::string image_copy = g_state->config.output_dir + "/cattle_report.jpg";
    if (!g_state->latest_image.empty() && CopyFile(g_state->latest_image, image_copy)) {
        attachment = image_copy;
    }
    std::ostringstream subject;
    subject << "Cattle count: " << headline;
    CattleEmailRequest request;
    request.python = g_state->config.python;
    request.script = g_state->config.email_script;
    request.config_path = g_state->config.config_path;
    request.body_path = body_path;
    request.subject = subject.str();
    request.attachment = attachment;
    USER_LOG_INFO("%s. %s", reason, subject.str().c_str());
    int status = SendCattleEmail(request, &error);
    if (status != 0) {
        USER_LOG_ERROR("Email was not sent (%s). The report is saved at %s", error.c_str(), body_path.c_str());
    } else {
        USER_LOG_INFO("Cattle report handed to the mailer. Copy: %s", body_path.c_str());
    }
    std::cout << report << std::endl;
    return status;
}

void PrintStatus() {
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    osal->MutexLock(g_state->counter_mutex);
    int confirmed = g_state->counter->ConfirmedCount();
    int candidates = g_state->counter->CandidateCount();
    int peak = g_state->counter->PeakInFrame();
    int headline = CattleHeadlineCount(*g_state->counter);
    CattleStats stats = g_state->counter->Stats();
    FrameResult last = g_state->last_result;
    CameraPose pose = g_state->last_pose;
    bool have_pose = g_state->have_pose;
    osal->MutexUnlock(g_state->counter_mutex);

    std::cout << "Cattle count: " << headline
              << "  confirmed: " << confirmed
              << "  unconfirmed: " << candidates
              << "  peak in frame: " << peak
              << "  counted frames: " << stats.frames_counted
              << "  skipped: " << stats.frames_skipped;
    if (last.skip != CATTLE_SKIP_NONE) {
        std::cout << "  last skip: " << CattleSkipReasonText(last.skip);
    }
    if (have_pose) {
        std::cout << "  agl_m: " << pose.alt_agl_m
                  << "  pitch: " << pose.gimbal_pitch_deg
                  << "  gps: " << (pose.gps_valid ? "yes" : "no");
    }
    if (!g_state->ground_set) {
        std::cout << "  ground altitude: not marked";
    }
    std::cout << std::endl;
}

#ifdef OPEN_CV_INSTALLED

bool EndsWith(const std::string &text, const char *suffix) {
    size_t length = std::strlen(suffix);
    return text.size() >= length && text.compare(text.size() - length, length, suffix) == 0;
}

bool FileNewerOrEqual(const std::string &path, const std::string &other) {
    struct stat left;
    struct stat right;
    if (stat(path.c_str(), &left) != 0 || stat(other.c_str(), &right) != 0) {
        return false;
    }
    if (left.st_size <= 0) {
        return false;
    }
    return left.st_mtime >= right.st_mtime;
}

bool GunzipTo(const std::string &gz_path, const std::string &dest) {
    std::string temporary = dest + ".tmp";
    std::string command = "gzip -dc " + ShellQuote(gz_path) + " > " + ShellQuote(temporary);
    int status = std::system(command.c_str());
    if (status != 0) {
        std::remove(temporary.c_str());
        return false;
    }
    if (std::rename(temporary.c_str(), dest.c_str()) != 0) {
        std::remove(temporary.c_str());
        return false;
    }
    return true;
}

std::string PrepareOnnxModel(const std::string &path) {
    if (!EndsWith(path, ".gz")) {
        return path;
    }
    std::string plain = path.substr(0, path.size() - 3);
    if (FileNewerOrEqual(plain, path)) {
        return plain;
    }
    if (GunzipTo(path, plain)) {
        USER_LOG_INFO("Inflated cattle ONNX model to %s", plain.c_str());
        return plain;
    }
    std::string fallback = g_state->config.output_dir + "/aerial_cattle_yolov8m.onnx";
    if (FileNewerOrEqual(fallback, path)) {
        return fallback;
    }
    EnsureDirectory(g_state->config.output_dir);
    if (GunzipTo(path, fallback)) {
        USER_LOG_INFO("Inflated cattle ONNX model to %s", fallback.c_str());
        return fallback;
    }
    USER_LOG_WARN("Could not inflate %s", path.c_str());
    return path;
}

std::vector<CattleDetection> ParseOnnxOutput(const cv::Mat &out, int input, float conf,
                                             const std::vector<int> &class_ids) {
    std::vector<CattleDetection> found;
    if (out.empty()) {
        return found;
    }
    cv::Mat flat = out.isContinuous() ? out : out.clone();
    int num_attrs = 0;
    int num_boxes = 0;
    bool channel_major = true;
    if (flat.dims == 3) {
        int d1 = flat.size[1];
        int d2 = flat.size[2];
        channel_major = d1 <= d2;
        num_attrs = channel_major ? d1 : d2;
        num_boxes = channel_major ? d2 : d1;
    } else if (flat.dims == 2) {
        channel_major = flat.rows <= flat.cols;
        num_attrs = channel_major ? flat.rows : flat.cols;
        num_boxes = channel_major ? flat.cols : flat.rows;
    } else {
        return found;
    }
    return ParseYoloV8(reinterpret_cast<const float *>(flat.ptr<float>()), num_attrs, num_boxes, channel_major,
                       input, input, input, input, conf, class_ids);
}

cv::Mat LetterboxCanvas(const cv::Mat &tile, const LetterboxFit &fit, int input) {
    cv::Mat canvas(input, input, CV_8UC3, cv::Scalar(114, 114, 114));
    if (fit.resized_w <= 0 || fit.resized_h <= 0) {
        return canvas;
    }
    cv::Mat resized;
    const cv::Mat *source = &tile;
    if (fit.resized_w != tile.cols || fit.resized_h != tile.rows) {
        cv::resize(tile, resized, cv::Size(fit.resized_w, fit.resized_h));
        source = &resized;
    }
    int px = static_cast<int>(fit.pad_x);
    int py = static_cast<int>(fit.pad_y);
    source->copyTo(canvas(cv::Rect(px, py, fit.resized_w, fit.resized_h)));
    return canvas;
}

bool AppendOnnxView(std::vector<CattleDetection> *found, const cv::Mat &view, float origin_x, float origin_y,
                    int input, float conf, const std::vector<int> &class_ids) {
    LetterboxFit fit = FitLetterbox(view.cols, view.rows, input);
    cv::Mat canvas = LetterboxCanvas(view, fit, input);
    cv::Mat blob =
        cv::dnn::blobFromImage(canvas, 1.0 / 255.0, cv::Size(input, input), cv::Scalar(), true, false);
    cv::Mat out;
    try {
        g_state->onnx.setInput(blob);
        out = g_state->onnx.forward();
    } catch (const cv::Exception &ex) {
        USER_LOG_WARN("Cattle ONNX forward failed: %s", ex.what());
        return false;
    }
    std::vector<CattleDetection> local = ParseOnnxOutput(out, input, conf, class_ids);
    for (size_t i = 0; i < local.size(); ++i) {
        MapLetterboxDetection(&local[i], fit, origin_x, origin_y);
        found->push_back(local[i]);
    }
    return true;
}

std::vector<CattleDetection> DetectOnnx(const cv::Mat &bgr) {
    std::vector<CattleDetection> found;
    int input = g_state->config.onnx_input > 0 ? g_state->config.onnx_input : 640;
    int stride = input * 3 / 4;
    if (stride < 1) {
        stride = 1;
    }
    float conf = static_cast<float>(g_state->config.counter.min_score);
    const std::vector<int> &class_ids = g_state->config.counter.class_ids;
    std::vector<int> xs = TileStarts(bgr.cols, input, stride);
    std::vector<int> ys = TileStarts(bgr.rows, input, stride);
    for (size_t yi = 0; yi < ys.size(); ++yi) {
        for (size_t xi = 0; xi < xs.size(); ++xi) {
            int x = xs[xi];
            int y = ys[yi];
            int w = std::min(input, bgr.cols - x);
            int h = std::min(input, bgr.rows - y);
            if (w <= 0 || h <= 0) {
                continue;
            }
            if (!AppendOnnxView(&found, bgr(cv::Rect(x, y, w, h)), static_cast<float>(x), static_cast<float>(y),
                                input, conf, class_ids)) {
                return found;
            }
        }
    }
    // Tiles keep a cow at the size it has in a full nadir photo. Letterboxing
    // the whole frame covers a tighter view, where that cow would be too large
    // inside one tile.
    if (bgr.cols > input || bgr.rows > input) {
        AppendOnnxView(&found, bgr, 0.0f, 0.0f, input, conf, class_ids);
    }
    return found;
}

std::vector<CattleDetection> DetectCoco(const cv::Mat &bgr) {
    std::vector<CattleDetection> found;
    std::shared_ptr<cv::Mat> image(new cv::Mat(bgr.clone()));
    std::vector<T_DjiLiveViewBoundingBox> boxes;
    g_state->yolo->Process(image, boxes);
    float score = 0.50f;
    if (g_state->config.counter.min_score > 0.50 && !g_state->logged_coco_score) {
        USER_LOG_WARN("The built-in COCO model only exposes boxes above 0.50. Use onnx_model for a real threshold.");
        g_state->logged_coco_score = true;
    }
    for (size_t i = 0; i < boxes.size(); ++i) {
        // The built-in network is COCO. Class 19 is cow even when the config
        // lists class 0 for the aerial ONNX model.
        if (boxes[i].type != 19) {
            continue;
        }
        CattleDetection detection;
        detection.cx = boxes[i].box.cx / 10000.0f * static_cast<float>(bgr.cols);
        detection.cy = boxes[i].box.cy / 10000.0f * static_cast<float>(bgr.rows);
        detection.w = boxes[i].box.w / 10000.0f * static_cast<float>(bgr.cols);
        detection.h = boxes[i].box.h / 10000.0f * static_cast<float>(bgr.rows);
        detection.score = score;
        detection.class_id = boxes[i].type;
        found.push_back(detection);
    }
    return found;
}

void SaveSnapshot(const cv::Mat &bgr, const FrameResult &result, int headline) {
    cv::Mat drawn = bgr.clone();
    for (size_t i = 0; i < result.hits.size(); ++i) {
        const FrameHit &hit = result.hits[i];
        cv::Rect rect(static_cast<int>(hit.cx - hit.w * 0.5f), static_cast<int>(hit.cy - hit.h * 0.5f),
                      static_cast<int>(hit.w), static_cast<int>(hit.h));
        cv::Scalar color = hit.confirmed ? cv::Scalar(0, 180, 0) : cv::Scalar(0, 200, 255);
        cv::rectangle(drawn, rect, color, 2);
        cv::putText(drawn, cv::format("%d", hit.id), cv::Point(rect.x, std::max(0, rect.y - 4)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
    cv::putText(drawn, cv::format("cattle %d", headline), cv::Point(16, 32), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(255, 255, 255), 2);
    if (!g_state->latest_image.empty()) {
        std::string temporary = g_state->latest_image + ".tmp";
        if (cv::imwrite(temporary, drawn)) {
            if (std::rename(temporary.c_str(), g_state->latest_image.c_str()) != 0) {
                std::remove(temporary.c_str());
            }
        }
    }
}

void *CattleDetectThread(void *arg) {
    (void)arg;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    int tilt_skips = 0;
    while (g_state->running.load()) {
        osal->MutexLock(g_state->image_mutex);
        if (g_state->images.empty()) {
            osal->MutexUnlock(g_state->image_mutex);
            osal->TaskSleepMs(10);
            continue;
        }
        while (g_state->images.size() > 1) {
            g_state->images.pop();
        }
        cv::Mat rgb = g_state->images.front();
        g_state->images.pop();
        osal->MutexUnlock(g_state->image_mutex);

        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        std::vector<CattleDetection> detections =
            g_state->use_onnx ? DetectOnnx(bgr) : DetectCoco(bgr);

        osal->MutexLock(g_state->counter_mutex);
        CameraPose pose;
        ReadPose(g_state->config, bgr.cols, bgr.rows, &pose);
        FrameResult result = g_state->counter->Update(pose, detections);
        g_state->last_result = result;
        g_state->last_pose = pose;
        g_state->have_pose = true;
        g_state->frames_seen += 1;
        int seen = g_state->frames_seen;
        int headline = CattleHeadlineCount(*g_state->counter);
        osal->MutexUnlock(g_state->counter_mutex);
        if (seen == 1 || seen % 30 == 0) {
            SaveSnapshot(bgr, result, headline);
        }
        if (result.skip == CATTLE_SKIP_TILT) {
            tilt_skips += 1;
            if (tilt_skips == 30) {
                USER_LOG_WARN("The gimbal does not look straight down. Press s and confirm pitch is near -90.");
            }
        }

        T_DjiLiveViewStandardMetaData *meta = 0;
        FillMeta(result, bgr.cols, bgr.rows, &meta);
        if (meta) {
            DjiLiveview_SendAiMetaToPilot(meta);
            T_DjiLiveViewStandardMetaData *queued = 0;
            FillMeta(result, bgr.cols, bgr.rows, &queued);
            if (queued) {
                PushMeta(queued);
            }
            free(meta);
        }
        if (seen % 30 == 0) {
            PrintStatus();
        }
    }
    return 0;
}

void ImageCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t len,
                   T_DjiLiveviewImageInfo image_info) {
    (void)position;
    if (!g_state || !buf || image_info.width == 0 || image_info.height == 0) {
        return;
    }
    cv::Mat rgb(image_info.height, image_info.width, CV_8UC3, const_cast<uint8_t *>(buf));
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    osal->MutexLock(g_state->image_mutex);
    if (g_state->images.size() > 8) {
        g_state->images.pop();
    }
    g_state->images.push(rgb.clone());
    osal->MutexUnlock(g_state->image_mutex);

    T_DjiLiveViewStandardMetaData *meta = PopMeta();
    DjiLiveview_EncodeAFrameToH264(buf, len, image_info, meta);
    if (meta) {
        free(meta);
    }
}

void EncoderCallback(const uint8_t *buf, uint32_t len) {
    T_DjiReturnCode code = DjiPayloadCamera_SendVideoStream(buf, len);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("failed to send the cattle live view, ret: 0x%08llX", code);
    }
}

bool InitDetector() {
    if (!g_state->config.onnx_model.empty()) {
        std::string model_path = PrepareOnnxModel(g_state->config.onnx_model);
        try {
            g_state->onnx = cv::dnn::readNetFromONNX(model_path);
        } catch (const cv::Exception &ex) {
            USER_LOG_WARN("Could not load %s (%s).", model_path.c_str(), ex.what());
            g_state->onnx = cv::dnn::Net();
        }
        if (!g_state->onnx.empty()) {
            g_state->use_onnx = true;
            USER_LOG_INFO("Loaded cattle ONNX model %s", model_path.c_str());
            return true;
        }
        USER_LOG_WARN("Could not load %s. Falling back to the built-in COCO cow class.",
                      model_path.c_str());
    }
    g_state->yolo = new ImageProcessorYolovFastest("cattle");
    if (g_state->yolo->Init() != 0) {
        USER_LOG_ERROR("Could not load the built-in detector.");
        delete g_state->yolo;
        g_state->yolo = 0;
        return false;
    }
    USER_LOG_INFO("Using the built-in COCO detector, class cow (19). An aerial cattle ONNX model is more accurate.");
    return true;
}

#endif  // OPEN_CV_INSTALLED

void DrainMeta() {
    while (!g_state->metas.empty()) {
        free(g_state->metas.front());
        g_state->metas.pop();
    }
}

const char *AircraftName(E_DjiAircraftType type) {
    if (type == DJI_AIRCRAFT_TYPE_M4T) {
        return "Matrice 4T";
    }
    if (type == DJI_AIRCRAFT_TYPE_M4TD) {
        return "Matrice 4TD";
    }
    return "DJI aircraft";
}

}  // namespace

void DjiUser_RunCattleCountSample(void) {
#ifndef OPEN_CV_INSTALLED
    USER_LOG_ERROR("The cattle counter needs OpenCV. Rebuild the Manifold 3 sample with OpenCV installed.");
    return;
#else
    SharedState state;
    g_state = &state;
    std::string config_path = FindCattleConfigPath();
    std::string error;
    if (!LoadCattleAppConfig(config_path, &state.config, &error)) {
        USER_LOG_WARN("Using default cattle settings (%s).", error.c_str());
        state.config = DefaultCattleAppConfig();
        state.config.config_path = config_path;
    } else {
        USER_LOG_INFO("Loaded cattle config %s", config_path.c_str());
    }
    if (state.config.ground_altitude_set) {
        state.ground_set = true;
        state.ground_altitude_m = state.config.ground_altitude_m;
    }
    state.counter = new CattleCounter(state.config.counter);
    EnsureDirectory(state.config.output_dir);
    state.latest_image = state.config.output_dir + "/latest.jpg";

    if (!InitDetector()) {
        delete state.counter;
        g_state = 0;
        return;
    }

    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    osal->MutexCreate(&state.image_mutex);
    osal->MutexCreate(&state.counter_mutex);
    osal->MutexCreate(&state.meta_mutex);

    bool fc_ready = DjiFcSubscription_Init() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    if (!fc_ready) {
        USER_LOG_WARN("Flight-data subscription was already started or did not start. Trying to read it anyway.");
    }
    bool sub_position = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                        DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 0) ==
                        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    bool sub_altitude = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                        DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 0) ==
                        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    bool sub_gimbal = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                                      DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, 0) ==
                      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    if (!sub_position || !sub_altitude || !sub_gimbal) {
        USER_LOG_WARN("Some flight topics did not subscribe. Press s and check gps, agl, and pitch before counting.");
    }

    T_DjiAircraftInfoBaseInfo aircraft = {};
    if (DjiAircraftInfo_GetBaseInfo(&aircraft) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        state.aircraft_name = AircraftName(aircraft.aircraftType);
        if (aircraft.aircraftType != DJI_AIRCRAFT_TYPE_M4T && aircraft.aircraftType != DJI_AIRCRAFT_TYPE_M4TD) {
            USER_LOG_WARN("This sample is set up for the Matrice 4T wide camera. Detected another aircraft.");
        }
    }

    const T_DjiDataChannelBandwidthProportionOfHighspeedChannel bandwidth = {10, 60, 30};
    T_DjiReturnCode code = DjiHighSpeedDataChannel_SetBandwidthProportion(bandwidth);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("High-speed channel init failed, ret: 0x%08llX", code);
    }
    bool liveview_ready = DjiLiveview_Init() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    const char *labels[] = {"cattle"};
    bool labels_ready = false;
    bool encoder_ready = false;
    bool stream_ready = false;
    T_DjiTaskHandle task = 0;
    E_DjiLiveViewCameraPosition position =
        static_cast<E_DjiLiveViewCameraPosition>(state.config.camera_position);
    E_DjiLiveViewCameraSource source = static_cast<E_DjiLiveViewCameraSource>(state.config.camera_source);

    if (!liveview_ready) {
        USER_LOG_ERROR("Live view init failed.");
        goto cleanup;
    }
    labels_ready = DjiLiveview_RegUserAiTargetLableList(1, labels) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    encoder_ready = DjiLiveview_RegEncoderCallback(EncoderCallback) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    if (!encoder_ready) {
        USER_LOG_ERROR("Encoder callback registration failed.");
        goto cleanup;
    }
    stream_ready = DjiLiveview_StartImageStream(position, source, PIXFMT_RGB_PACKED, ImageCallback) ==
                   DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    if (!stream_ready) {
        USER_LOG_ERROR("Could not start the camera stream. camera_source 1 is the Matrice 4T wide view.");
        goto cleanup;
    }

    state.running.store(true);
    if (osal->TaskCreate("cattleCount", CattleDetectThread, 2 * 1024 * 1024, 0, &task) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Could not start the cattle counting task.");
        state.running.store(false);
        goto cleanup;
    }

    std::cout
        << "\nCattle counter is watching the wide camera.\n"
        << "Point the gimbal straight down and fly about 60 to 100 m above the pasture.\n"
        << "Mark the ground altitude before takeoff so the height above the herd is right.\n"
        << "[g] mark ground altitude    [s] show the count\n"
        << "[e] email the count         [r] reset the count\n"
        << "[q] stop and email\n"
        << std::endl;

    while (state.running.load()) {
        char command = 0;
        if (!(std::cin >> command)) {
            break;
        }
        if (command == 'g' || command == 'G') {
            double altitude_m = 0.0;
            if (ReadFusedAltitude(&altitude_m)) {
                osal->MutexLock(state.counter_mutex);
                state.ground_set = true;
                state.ground_altitude_m = altitude_m;
                osal->MutexUnlock(state.counter_mutex);
                USER_LOG_INFO("Ground altitude marked at %.2f m.", altitude_m);
            } else {
                USER_LOG_ERROR("Ground altitude is not available yet.");
            }
        } else if (command == 's' || command == 'S') {
            PrintStatus();
        } else if (command == 'e' || command == 'E') {
            EmailCount("Email requested");
        } else if (command == 'r' || command == 'R') {
            osal->MutexLock(state.counter_mutex);
            state.counter->Reset();
            osal->MutexUnlock(state.counter_mutex);
            USER_LOG_INFO("Cattle tracks cleared.");
        } else if (command == 'q' || command == 'Q') {
            break;
        }
    }

cleanup:
    state.running.store(false);
    osal->TaskSleepMs(200);
    if (task) {
        osal->TaskDestroy(task);
    }
    if (stream_ready) {
        DjiLiveview_StopImageStream(position, source);
    }
    if (encoder_ready) {
        DjiLiveview_UnregEncoderCallback();
    }
    if (labels_ready) {
        DjiLiveview_UnregUserAiTargetLableList();
    }
    if (liveview_ready) {
        DjiLiveview_Deinit();
    }
    if (sub_position) {
        DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED);
    }
    if (sub_altitude) {
        DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED);
    }
    if (sub_gimbal) {
        DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
    }
    if (fc_ready) {
        DjiFcSubscription_DeInit();
    }
    if (state.config.email_on_exit && stream_ready) {
        EmailCount("Count finished");
    }
    DrainMeta();
    osal->MutexDestroy(state.image_mutex);
    osal->MutexDestroy(state.counter_mutex);
    osal->MutexDestroy(state.meta_mutex);
    delete state.yolo;
    delete state.counter;
    g_state = 0;
#endif
}

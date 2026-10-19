#include "cattle_config.hpp"
#include "cattle_counter.hpp"
#include "cattle_report.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

int g_failures = 0;

void Expect(bool condition, const char *text, int line) {
    if (!condition) {
        std::cerr << "FAIL " << line << ": " << text << std::endl;
        g_failures += 1;
    }
}

#define EXPECT(cond) Expect((cond), #cond, __LINE__)

const double kMetersPerDeg = 111320.0;

CattleOptions TestOptions() {
    CattleOptions options = DefaultCattleOptions();
    options.hfov_deg = 90.0;
    options.min_alt_m = 5.0;
    options.min_observations = 2;
    options.class_ids.clear();
    options.class_ids.push_back(19);
    return options;
}

CameraPose TestPose(double time_s, double north_m, double east_m) {
    CameraPose pose;
    pose.time_s = time_s;
    pose.lat_deg = north_m / kMetersPerDeg;
    pose.lon_deg = east_m / kMetersPerDeg;
    pose.alt_agl_m = 10.0;
    pose.gimbal_pitch_deg = -90.0;
    pose.gimbal_yaw_deg = 0.0;
    pose.width = 100;
    pose.height = 100;
    pose.gps_valid = true;
    return pose;
}

CattleDetection Cow(float cx, float cy) {
    CattleDetection detection;
    detection.cx = cx;
    detection.cy = cy;
    detection.w = 10.0f;
    detection.h = 10.0f;
    detection.score = 0.9f;
    detection.class_id = 19;
    return detection;
}

double NorthMeters(double lat_deg, double origin_lat) {
    return (lat_deg - origin_lat) * kMetersPerDeg;
}

double EastMeters(double lon_deg, double origin_lon, double origin_lat) {
    return (lon_deg - origin_lon) * kMetersPerDeg * std::cos(origin_lat * 3.14159265358979323846 / 180.0);
}

void TestHorizontalFov() {
    CattleOptions options = DefaultCattleOptions();
    double hfov = CattleCounter::HorizontalFovDeg(options);
    EXPECT(hfov > 69.4);
    EXPECT(hfov < 69.9);
}

void TestProjection() {
    CattleOptions options = TestOptions();
    CameraPose pose = TestPose(0.0, 0.0, 0.0);
    double lat = 1.0;
    double lon = 1.0;
    double mx = 0.0;
    double my = 0.0;
    CattleSkipReason skip = CATTLE_SKIP_GPS;
    EXPECT(CattleCounter::ProjectToGround(options, pose, 49.5f, 49.5f, &lat, &lon, &mx, &my, &skip));
    EXPECT(skip == CATTLE_SKIP_NONE);
    EXPECT(std::fabs(lat) < 1e-9);
    EXPECT(std::fabs(lon) < 1e-9);
    EXPECT(std::fabs(mx - 0.2) < 1e-9);

    EXPECT(CattleCounter::ProjectToGround(options, pose, 74.5f, 49.5f, &lat, &lon, 0, 0, &skip));
    EXPECT(std::fabs(EastMeters(lon, 0.0, 0.0) - 5.0) < 0.05);
    EXPECT(std::fabs(NorthMeters(lat, 0.0)) < 0.05);

    EXPECT(CattleCounter::ProjectToGround(options, pose, 49.5f, 24.5f, &lat, &lon, 0, 0, &skip));
    EXPECT(std::fabs(NorthMeters(lat, 0.0) - 5.0) < 0.05);
    EXPECT(std::fabs(EastMeters(lon, 0.0, 0.0)) < 0.05);

    pose.gimbal_yaw_deg = 90.0;
    EXPECT(CattleCounter::ProjectToGround(options, pose, 49.5f, 24.5f, &lat, &lon, 0, 0, &skip));
    EXPECT(std::fabs(EastMeters(lon, 0.0, 0.0) - 5.0) < 0.05);
    EXPECT(std::fabs(NorthMeters(lat, 0.0)) < 0.05);

    pose = TestPose(0.0, 0.0, 0.0);
    pose.lat_deg = 40.0;
    pose.lon_deg = 10.0;
    EXPECT(CattleCounter::ProjectToGround(options, pose, 74.5f, 49.5f, &lat, &lon, 0, 0, &skip));
    EXPECT(std::fabs(EastMeters(lon, 10.0, 40.0) - 5.0) < 0.05);
    EXPECT(std::fabs(NorthMeters(lat, 40.0)) < 0.05);

    pose = TestPose(0.0, 0.0, 0.0);
    pose.gimbal_pitch_deg = -40.0;
    EXPECT(!CattleCounter::ProjectToGround(options, pose, 49.5f, 49.5f, &lat, &lon, 0, 0, &skip));
    EXPECT(skip == CATTLE_SKIP_TILT);

    pose = TestPose(0.0, 0.0, 0.0);
    pose.alt_agl_m = 2.0;
    EXPECT(!CattleCounter::ProjectToGround(options, pose, 49.5f, 49.5f, &lat, &lon, 0, 0, &skip));
    EXPECT(skip == CATTLE_SKIP_ALTITUDE);
}

void TestSurveyDoesNotDoubleCount() {
    CattleOptions options = TestOptions();
    CattleCounter counter(options);
    std::vector<CattleDetection> first;
    first.push_back(Cow(49.5f, 49.5f));
    first.push_back(Cow(74.5f, 49.5f));

    FrameResult frame = counter.Update(TestPose(0.0, 0.0, 0.0), first);
    EXPECT(frame.used_gps);
    EXPECT(frame.in_frame == 2);
    EXPECT(counter.ConfirmedCount() == 0);
    EXPECT(counter.CandidateCount() == 2);

    frame = counter.Update(TestPose(0.4, 0.0, 0.0), first);
    EXPECT(counter.ConfirmedCount() == 2);
    EXPECT(frame.hits.size() == 2);
    EXPECT(frame.hits[0].confirmed);
    EXPECT(frame.hits[1].confirmed);

    std::vector<CattleDetection> revisit;
    revisit.push_back(Cow(49.5f, 69.5f));
    revisit.push_back(Cow(74.5f, 69.5f));
    frame = counter.Update(TestPose(2.0, 4.0, 0.0), revisit);
    EXPECT(frame.in_frame == 2);
    EXPECT(counter.ConfirmedCount() == 2);
    EXPECT(counter.CandidateCount() == 0);

    std::vector<CattleDetection> extra;
    extra.push_back(Cow(9.5f, 49.5f));
    counter.Update(TestPose(3.0, 0.0, 0.0), extra);
    counter.Update(TestPose(3.4, 0.0, 0.0), extra);
    EXPECT(counter.ConfirmedCount() == 3);

    std::string report = BuildCattleReport(counter, "Matrice 4T", "2026-10-19 00:00:00 UTC");
    EXPECT(report.find("Cattle count: 3") != std::string::npos);
    EXPECT(report.find("Confirmed cattle: 3") != std::string::npos);
    EXPECT(CattleHeadlineCount(counter) == 3);
}

void TestRejectsNoiseAndTilt() {
    CattleOptions options = TestOptions();
    options.min_observations = 3;
    CattleCounter counter(options);
    std::vector<CattleDetection> tiny;
    tiny.push_back(Cow(49.5f, 49.5f));
    tiny[0].w = 1.0f;
    tiny[0].h = 1.0f;
    counter.Update(TestPose(0.0, 0.0, 0.0), tiny);
    EXPECT(counter.ConfirmedCount() == 0);
    EXPECT(counter.CandidateCount() == 0);

    std::vector<CattleDetection> other;
    other.push_back(Cow(49.5f, 49.5f));
    other[0].class_id = 0;
    counter.Update(TestPose(0.2, 0.0, 0.0), other);
    EXPECT(counter.CandidateCount() == 0);

    std::vector<CattleDetection> low;
    low.push_back(Cow(49.5f, 49.5f));
    low[0].score = 0.1f;
    counter.Update(TestPose(0.3, 0.0, 0.0), low);
    EXPECT(counter.CandidateCount() == 0);

    CameraPose tilted = TestPose(0.4, 0.0, 0.0);
    tilted.gimbal_pitch_deg = -40.0;
    FrameResult frame = counter.Update(tilted, std::vector<CattleDetection>(1, Cow(49.5f, 49.5f)));
    EXPECT(frame.skip == CATTLE_SKIP_TILT);
    EXPECT(counter.Stats().skip_tilt == 1);

    counter.Update(TestPose(1.0, 0.0, 0.0), std::vector<CattleDetection>(1, Cow(49.5f, 49.5f)));
    EXPECT(counter.ConfirmedCount() == 0);
    EXPECT(counter.CandidateCount() == 1);
}

void TestWalkingCowStaysOne() {
    CattleOptions options = TestOptions();
    CattleCounter counter(options);
    counter.Update(TestPose(0.0, 0.0, 0.0), std::vector<CattleDetection>(1, Cow(49.5f, 49.5f)));
    counter.Update(TestPose(2.0, 0.0, 0.0), std::vector<CattleDetection>(1, Cow(64.5f, 49.5f)));
    EXPECT(counter.ConfirmedCount() == 1);
    EXPECT(counter.Fixes().size() == 1);
}

void TestPackedBoxesDedup() {
    CattleOptions options = TestOptions();
    CattleCounter counter(options);
    std::vector<CattleDetection> pair;
    pair.push_back(Cow(49.5f, 49.5f));
    pair.push_back(Cow(54.5f, 49.5f));
    FrameResult frame = counter.Update(TestPose(0.0, 0.0, 0.0), pair);
    EXPECT(frame.in_frame == 1);
}

void TestNoGpsDoesNotInventASurvey() {
    CattleOptions options = TestOptions();
    CattleCounter counter(options);
    CameraPose pose = TestPose(0.0, 0.0, 0.0);
    pose.gps_valid = false;
    FrameResult frame = counter.Update(pose, std::vector<CattleDetection>(1, Cow(49.5f, 49.5f)));
    EXPECT(frame.skip == CATTLE_SKIP_GPS);
    EXPECT(counter.ConfirmedCount() == 0);
    EXPECT(!counter.SurveyUsedGps());
    std::string report = BuildCattleReport(counter, "Matrice 4T", "now");
    EXPECT(report.find("GPS survey: not available") != std::string::npos);
    EXPECT(CattleHeadlineCount(counter) == 0);
}

void TestImageTracksWhenEnabled() {
    CattleOptions options = TestOptions();
    options.allow_image_tracks = true;
    CattleCounter counter(options);
    CameraPose pose = TestPose(0.0, 0.0, 0.0);
    pose.gps_valid = false;
    counter.Update(pose, std::vector<CattleDetection>(1, Cow(40.0f, 40.0f)));
    pose.time_s = 0.5;
    counter.Update(pose, std::vector<CattleDetection>(1, Cow(42.0f, 41.0f)));
    EXPECT(counter.ImageTrackCount() == 1);
    pose.time_s = 1.0;
    counter.Update(pose, std::vector<CattleDetection>(1, Cow(80.0f, 80.0f)));
    EXPECT(counter.ImageTrackCount() == 2);
    EXPECT(CattleHeadlineCount(counter) == 2);
}

void TestNmsAndYolo() {
    std::vector<CattleDetection> boxes;
    boxes.push_back(Cow(20, 20));
    boxes.push_back(Cow(21, 20));
    boxes[1].score = 0.4f;
    boxes.push_back(Cow(80, 80));
    std::vector<CattleDetection> kept = NonMaxSuppression(boxes, 0.5f);
    EXPECT(kept.size() == 2);

    float data[10];
    for (int i = 0; i < 10; ++i) {
        data[i] = 0.0f;
    }
    const int num_boxes = 2;
    const int num_attrs = 5;
    data[0 * num_boxes + 0] = 320.0f;
    data[1 * num_boxes + 0] = 320.0f;
    data[2 * num_boxes + 0] = 40.0f;
    data[3 * num_boxes + 0] = 20.0f;
    data[4 * num_boxes + 0] = 0.10f;
    data[0 * num_boxes + 1] = 100.0f;
    data[1 * num_boxes + 1] = 200.0f;
    data[2 * num_boxes + 1] = 30.0f;
    data[3 * num_boxes + 1] = 30.0f;
    data[4 * num_boxes + 1] = 0.90f;
    std::vector<int> classes;
    classes.push_back(0);
    std::vector<CattleDetection> parsed =
        ParseYoloV8(data, num_attrs, num_boxes, true, 640, 640, 1280, 640, 0.45f, classes);
    EXPECT(parsed.size() == 1);
    EXPECT(std::fabs(parsed[0].cx - 200.0f) < 0.01f);
    EXPECT(std::fabs(parsed[0].cy - 200.0f) < 0.01f);
    EXPECT(std::fabs(parsed[0].w - 60.0f) < 0.01f);
    EXPECT(std::fabs(parsed[0].h - 30.0f) < 0.01f);

    classes[0] = 1;
    parsed = ParseYoloV8(data, num_attrs, num_boxes, true, 640, 640, 1280, 640, 0.45f, classes);
    EXPECT(parsed.empty());

    float box_major[6] = {10.0f, 12.0f, 8.0f, 8.0f, 0.2f, 0.8f};
    std::vector<int> cow_class;
    cow_class.push_back(1);
    parsed = ParseYoloV8(box_major, 6, 1, false, 64, 64, 64, 64, 0.45f, cow_class);
    EXPECT(parsed.size() == 1);
    EXPECT(parsed[0].class_id == 1);
    EXPECT(std::fabs(parsed[0].score - 0.8f) < 0.001f);

    LetterboxFit fit = FitLetterbox(2000, 1500, 640);
    EXPECT(std::fabs(fit.scale - 0.32f) < 1e-6f);
    EXPECT(fit.resized_w == 640);
    EXPECT(fit.resized_h == 480);
    EXPECT(std::fabs(fit.pad_x) < 1e-6f);
    EXPECT(std::fabs(fit.pad_y - 80.0f) < 1e-6f);
    CattleDetection mapped;
    mapped.cx = 100.0f;
    mapped.cy = 180.0f;
    mapped.w = 32.0f;
    mapped.h = 16.0f;
    mapped.score = 0.5f;
    mapped.class_id = 0;
    MapLetterboxDetection(&mapped, fit, 480.0f, 0.0f);
    EXPECT(std::fabs(mapped.cx - (100.0f / 0.32f + 480.0f)) < 0.01f);
    EXPECT(std::fabs(mapped.cy - ((180.0f - 80.0f) / 0.32f)) < 0.01f);
    EXPECT(std::fabs(mapped.w - (32.0f / 0.32f)) < 0.01f);
    EXPECT(std::fabs(mapped.h - (16.0f / 0.32f)) < 0.01f);

    std::vector<int> rows = TileStarts(1080, 640, 480);
    EXPECT(rows.size() == 2);
    EXPECT(rows[0] == 0);
    EXPECT(rows[1] == 440);
    std::vector<int> cols = TileStarts(1920, 640, 480);
    EXPECT(cols.size() == 4);
    EXPECT(cols[0] == 0);
    EXPECT(cols[1] == 480);
    EXPECT(cols[2] == 960);
    EXPECT(cols[3] == 1280);
    std::vector<int> small = TileStarts(480, 640, 480);
    EXPECT(small.size() == 1);
    EXPECT(small[0] == 0);
}

void TestConfigAndCommand() {
    CattleAppConfig loaded;
    std::string error;
    std::string example = "config.example.json";
    if (!LoadCattleAppConfig(example, &loaded, &error)) {
        example = "cattle/config.example.json";
    }
    EXPECT(LoadCattleAppConfig(example, &loaded, &error));
    if (!error.empty() && g_failures) {
        std::cerr << error << std::endl;
    }
    EXPECT(loaded.counter.class_ids.size() == 1);
    EXPECT(loaded.counter.class_ids[0] == 0);
    std::string model = "models/aerial_cattle_yolov8m.onnx.gz";
    if (example.find('/') != std::string::npos) {
        model = "cattle/models/aerial_cattle_yolov8m.onnx.gz";
    }
    EXPECT(loaded.onnx_model == model);
    EXPECT(loaded.counter.dfov_deg == 82.0);
    EXPECT(loaded.email_on_exit);
    EXPECT(!loaded.ground_altitude_set);
    EXPECT(loaded.camera_source == 1);
    EXPECT(loaded.counter.min_observations == 3);

    const char *path = "/tmp/cattle-config-test.json";
    std::ofstream output(path);
    output << "{\n"
           << "\"class_ids\": [0],\n"
           << "\"email_on_exit\": false,\n"
           << "\"ground_altitude_m\": 123.5,\n"
           << "\"smtp_password\": \"a\\\"b\\\\c\",\n"
           << "\"onnx_model\": \"/tmp/cattle.onnx\"\n"
           << "}\n";
    output.close();
    EXPECT(LoadCattleAppConfig(path, &loaded, &error));
    EXPECT(loaded.counter.class_ids.size() == 1);
    EXPECT(loaded.counter.class_ids[0] == 0);
    EXPECT(!loaded.email_on_exit);
    EXPECT(loaded.ground_altitude_set);
    EXPECT(std::fabs(loaded.ground_altitude_m - 123.5) < 1e-6);
    EXPECT(loaded.onnx_model == "/tmp/cattle.onnx");
    EXPECT(loaded.counter.dfov_deg == 82.0);

    const char *relative_path = "/tmp/cattle-config-rel.json";
    std::ofstream relative(relative_path);
    relative << "{\n\"onnx_model\": \"models/foo.onnx\"\n}\n";
    relative.close();
    EXPECT(LoadCattleAppConfig(relative_path, &loaded, &error));
    EXPECT(loaded.onnx_model == "/tmp/models/foo.onnx");

    std::string shipped = "models/aerial_cattle_yolov8m.onnx.gz";
    std::ifstream packed(shipped.c_str(), std::ios::binary);
    if (!packed) {
        shipped = "cattle/models/aerial_cattle_yolov8m.onnx.gz";
        packed.open(shipped.c_str(), std::ios::binary);
    }
    EXPECT(packed.good());
    char magic[2] = {0, 0};
    packed.read(magic, 2);
    EXPECT(static_cast<unsigned char>(magic[0]) == 0x1f);
    EXPECT(static_cast<unsigned char>(magic[1]) == 0x8b);
    packed.seekg(0, std::ios::end);
    EXPECT(packed.tellg() > 50 * 1000 * 1000);

    setenv("CATTLE_CONFIG", example.c_str(), 1);
    EXPECT(FindCattleConfigPath() == example);

    CattleEmailRequest request;
    request.python = "python3";
    request.script = "cattle/send_report.py";
    request.config_path = "my config.json";
    request.body_path = "it's a report.txt";
    request.subject = "Cattle count: 4";
    request.attachment = "";
    std::string command = BuildEmailCommand(request);
    EXPECT(command.find("'my config.json'") != std::string::npos);
    EXPECT(command.find("'it'\\''s a report.txt'") != std::string::npos);
    EXPECT(command.find("--attach") == std::string::npos);

    EXPECT(!LoadCattleAppConfig("/tmp/does-not-exist-cattle.json", &loaded, &error));
    EXPECT(error.find("could not open") != std::string::npos);
}

}  // namespace

int main() {
    TestHorizontalFov();
    TestProjection();
    TestSurveyDoesNotDoubleCount();
    TestRejectsNoiseAndTilt();
    TestWalkingCowStaysOne();
    TestPackedBoxesDedup();
    TestNoGpsDoesNotInventASurvey();
    TestImageTracksWhenEnabled();
    TestNmsAndYolo();
    TestConfigAndCommand();
    if (g_failures != 0) {
        std::cerr << g_failures << " failure(s)" << std::endl;
        return 1;
    }
    std::cout << "cattle counter tests passed" << std::endl;
    return 0;
}

#include "cattle_counter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {

const double kPi = 3.14159265358979323846;
const double kMetersPerDeg = 111320.0;

double DegToRad(double deg) {
    return deg * kPi / 180.0;
}

double ClampCos(double deg) {
    double c = std::cos(DegToRad(deg));
    if (c < 0.05) {
        c = 0.05;
    }
    return c;
}

float IoU(const CattleDetection &a, const CattleDetection &b) {
    float ax1 = a.cx - a.w * 0.5f;
    float ay1 = a.cy - a.h * 0.5f;
    float ax2 = ax1 + a.w;
    float ay2 = ay1 + a.h;
    float bx1 = b.cx - b.w * 0.5f;
    float by1 = b.cy - b.h * 0.5f;
    float bx2 = bx1 + b.w;
    float by2 = by1 + b.h;
    float ix1 = std::max(ax1, bx1);
    float iy1 = std::max(ay1, by1);
    float ix2 = std::min(ax2, bx2);
    float iy2 = std::min(ay2, by2);
    float iw = ix2 - ix1;
    float ih = iy2 - iy1;
    if (iw <= 0.0f || ih <= 0.0f) {
        return 0.0f;
    }
    float inter = iw * ih;
    float uni = a.w * a.h + b.w * b.h - inter;
    if (uni <= 0.0f) {
        return 0.0f;
    }
    return inter / uni;
}

float At(const float *data, int num_attrs, int num_boxes, bool channel_major, int attr, int box) {
    if (channel_major) {
        return data[attr * num_boxes + box];
    }
    return data[box * num_attrs + attr];
}

struct ProjectedDet {
    CattleDetection box;
    double lat_deg;
    double lon_deg;
    double length_m;
};

}  // namespace

CattleOptions DefaultCattleOptions() {
    CattleOptions options;
    options.hfov_deg = 0.0;
    options.dfov_deg = 82.0;
    options.sensor_width = 4.0;
    options.sensor_height = 3.0;
    options.nadir_pitch_deg = -90.0;
    options.max_tilt_from_nadir_deg = 18.0;
    options.match_radius_m = 3.5;
    options.max_animal_speed_m_s = 1.5;
    options.max_motion_m = 6.0;
    options.dedup_radius_m = 1.5;
    options.min_length_m = 0.5;
    options.max_length_m = 4.5;
    options.min_score = 0.45;
    options.min_alt_m = 8.0;
    options.max_alt_m = 120.0;
    options.min_observations = 3;
    options.nms_iou = 0.5;
    options.allow_image_tracks = false;
    options.class_ids.push_back(19);
    return options;
}

const char *CattleSkipReasonText(CattleSkipReason reason) {
    switch (reason) {
        case CATTLE_SKIP_NONE:
            return "";
        case CATTLE_SKIP_TILT:
            return "gimbal is not looking straight down";
        case CATTLE_SKIP_ALTITUDE:
            return "altitude is outside the counting range";
        case CATTLE_SKIP_GPS:
            return "GPS fix or ground altitude is missing";
        case CATTLE_SKIP_IMAGE:
            return "image size is invalid";
        default:
            return "frame skipped";
    }
}

double CattleCounter::HorizontalFovDeg(const CattleOptions &options) {
    if (options.hfov_deg > 0.0) {
        return options.hfov_deg;
    }
    double diag = std::sqrt(options.sensor_width * options.sensor_width +
                            options.sensor_height * options.sensor_height);
    if (diag <= 0.0) {
        return 70.0;
    }
    double tan_half = std::tan(DegToRad(options.dfov_deg) * 0.5) * (options.sensor_width / diag);
    return 2.0 * std::atan(tan_half) * 180.0 / kPi;
}

double CattleCounter::MetersBetween(double lat1, double lon1, double lat2, double lon2) {
    double north = (lat2 - lat1) * kMetersPerDeg;
    double east = (lon2 - lon1) * kMetersPerDeg * std::cos(DegToRad(lat1));
    return std::sqrt(north * north + east * east);
}

bool CattleCounter::ProjectToGround(const CattleOptions &options,
                                    const CameraPose &pose,
                                    float cx,
                                    float cy,
                                    double *lat_deg,
                                    double *lon_deg,
                                    double *meters_per_px_x,
                                    double *meters_per_px_y,
                                    CattleSkipReason *skip) {
    if (skip) {
        *skip = CATTLE_SKIP_NONE;
    }
    if (pose.width < 2 || pose.height < 2) {
        if (skip) {
            *skip = CATTLE_SKIP_IMAGE;
        }
        return false;
    }
    if (!pose.gps_valid) {
        if (skip) {
            *skip = CATTLE_SKIP_GPS;
        }
        return false;
    }
    double tilt = pose.gimbal_pitch_deg - options.nadir_pitch_deg;
    if (std::fabs(tilt) > options.max_tilt_from_nadir_deg) {
        if (skip) {
            *skip = CATTLE_SKIP_TILT;
        }
        return false;
    }
    if (pose.alt_agl_m < options.min_alt_m || pose.alt_agl_m > options.max_alt_m) {
        if (skip) {
            *skip = CATTLE_SKIP_ALTITUDE;
        }
        return false;
    }

    double hfov = DegToRad(HorizontalFovDeg(options));
    double tilt_rad = DegToRad(tilt);
    double cos_tilt = std::cos(tilt_rad);
    if (cos_tilt < 0.25) {
        if (skip) {
            *skip = CATTLE_SKIP_TILT;
        }
        return false;
    }
    double slant = pose.alt_agl_m / cos_tilt;
    double ground_width = 2.0 * slant * std::tan(hfov * 0.5);
    double m_per_px_x = ground_width / static_cast<double>(pose.width);
    // The live view uses square pixels, so the ground sample distance is the
    // same horizontally and vertically.
    double m_per_px_y = m_per_px_x;

    double center_x = (static_cast<double>(pose.width) - 1.0) * 0.5;
    double center_y = (static_cast<double>(pose.height) - 1.0) * 0.5;
    double dx = static_cast<double>(cx) - center_x;
    double dy = static_cast<double>(cy) - center_y;

    // Image up is the gimbal forward direction. Pitch above nadir looks forward.
    double forward_m = pose.alt_agl_m * std::tan(tilt_rad) - dy * m_per_px_y;
    double right_m = dx * m_per_px_x;

    double yaw = DegToRad(pose.gimbal_yaw_deg);
    double east = forward_m * std::sin(yaw) + right_m * std::cos(yaw);
    double north = forward_m * std::cos(yaw) - right_m * std::sin(yaw);

    if (lat_deg) {
        *lat_deg = pose.lat_deg + north / kMetersPerDeg;
    }
    if (lon_deg) {
        *lon_deg = pose.lon_deg + east / (kMetersPerDeg * ClampCos(pose.lat_deg));
    }
    if (meters_per_px_x) {
        *meters_per_px_x = m_per_px_x;
    }
    if (meters_per_px_y) {
        *meters_per_px_y = m_per_px_y;
    }
    return true;
}

std::vector<CattleDetection> ParseYoloV8(const float *data,
                                         int num_attrs,
                                         int num_boxes,
                                         bool channel_major,
                                         int input_w,
                                         int input_h,
                                         int frame_w,
                                         int frame_h,
                                         float conf_thres,
                                         const std::vector<int> &class_ids) {
    std::vector<CattleDetection> found;
    if (!data || num_attrs < 5 || num_boxes <= 0 || input_w <= 0 || input_h <= 0 || frame_w <= 0 || frame_h <= 0) {
        return found;
    }

    float max_center = 0.0f;
    for (int box = 0; box < num_boxes; ++box) {
        max_center = std::max(max_center, std::fabs(At(data, num_attrs, num_boxes, channel_major, 0, box)));
        max_center = std::max(max_center, std::fabs(At(data, num_attrs, num_boxes, channel_major, 1, box)));
    }
    bool normalized = max_center <= 1.5f;
    float sx = static_cast<float>(frame_w) / static_cast<float>(input_w);
    float sy = static_cast<float>(frame_h) / static_cast<float>(input_h);

    for (int box = 0; box < num_boxes; ++box) {
        float cx = At(data, num_attrs, num_boxes, channel_major, 0, box);
        float cy = At(data, num_attrs, num_boxes, channel_major, 1, box);
        float w = At(data, num_attrs, num_boxes, channel_major, 2, box);
        float h = At(data, num_attrs, num_boxes, channel_major, 3, box);
        if (normalized) {
            cx *= static_cast<float>(input_w);
            cy *= static_cast<float>(input_h);
            w *= static_cast<float>(input_w);
            h *= static_cast<float>(input_h);
        }
        int best_class = 0;
        float best_score = At(data, num_attrs, num_boxes, channel_major, 4, box);
        for (int attr = 5; attr < num_attrs; ++attr) {
            float score = At(data, num_attrs, num_boxes, channel_major, attr, box);
            if (score > best_score) {
                best_score = score;
                best_class = attr - 4;
            }
        }
        if (best_score < conf_thres || w <= 1.0f || h <= 1.0f) {
            continue;
        }
        if (!class_ids.empty()) {
            bool allowed = false;
            for (size_t i = 0; i < class_ids.size(); ++i) {
                if (class_ids[i] == best_class) {
                    allowed = true;
                    break;
                }
            }
            if (!allowed) {
                continue;
            }
        }
        CattleDetection det;
        det.cx = cx * sx;
        det.cy = cy * sy;
        det.w = w * sx;
        det.h = h * sy;
        det.score = best_score;
        det.class_id = best_class;
        found.push_back(det);
    }
    return found;
}

LetterboxFit FitLetterbox(int src_w, int src_h, int dst) {
    LetterboxFit fit;
    fit.scale = 1.0f;
    fit.pad_x = 0.0f;
    fit.pad_y = 0.0f;
    fit.resized_w = src_w;
    fit.resized_h = src_h;
    if (src_w <= 0 || src_h <= 0 || dst <= 0) {
        return fit;
    }
    float scale = std::min(static_cast<float>(dst) / static_cast<float>(src_w),
                           static_cast<float>(dst) / static_cast<float>(src_h));
    int nw = static_cast<int>(std::lround(static_cast<float>(src_w) * scale));
    int nh = static_cast<int>(std::lround(static_cast<float>(src_h) * scale));
    if (nw < 1) {
        nw = 1;
    }
    if (nh < 1) {
        nh = 1;
    }
    if (nw > dst) {
        nw = dst;
    }
    if (nh > dst) {
        nh = dst;
    }
    fit.scale = scale;
    fit.pad_x = static_cast<float>((dst - nw) / 2);
    fit.pad_y = static_cast<float>((dst - nh) / 2);
    fit.resized_w = nw;
    fit.resized_h = nh;
    return fit;
}

void MapLetterboxDetection(CattleDetection *detection,
                           const LetterboxFit &fit,
                           float origin_x,
                           float origin_y) {
    if (!detection || fit.scale <= 0.0f) {
        return;
    }
    detection->cx = (detection->cx - fit.pad_x) / fit.scale + origin_x;
    detection->cy = (detection->cy - fit.pad_y) / fit.scale + origin_y;
    detection->w /= fit.scale;
    detection->h /= fit.scale;
}

std::vector<int> TileStarts(int length, int tile, int stride) {
    std::vector<int> starts;
    if (length <= 0 || tile <= 0) {
        return starts;
    }
    if (stride < 1) {
        stride = tile;
    }
    if (length <= tile) {
        starts.push_back(0);
        return starts;
    }
    for (int pos = 0; pos + tile < length; pos += stride) {
        starts.push_back(pos);
    }
    int last = length - tile;
    if (starts.empty() || starts.back() != last) {
        starts.push_back(last);
    }
    return starts;
}

std::vector<CattleDetection> NonMaxSuppression(const std::vector<CattleDetection> &boxes, float iou_thres) {
    std::vector<CattleDetection> ordered = boxes;
    std::sort(ordered.begin(), ordered.end(), [](const CattleDetection &a, const CattleDetection &b) {
        return a.score > b.score;
    });
    std::vector<char> removed(ordered.size(), 0);
    std::vector<CattleDetection> kept;
    for (size_t i = 0; i < ordered.size(); ++i) {
        if (removed[i]) {
            continue;
        }
        kept.push_back(ordered[i]);
        for (size_t j = i + 1; j < ordered.size(); ++j) {
            if (!removed[j] && IoU(ordered[i], ordered[j]) >= iou_thres) {
                removed[j] = 1;
            }
        }
    }
    return kept;
}

CattleCounter::CattleCounter(const CattleOptions &options)
    : options_(options), next_id_(1), peak_in_frame_(0), survey_used_gps_(false) {
    stats_.frames_counted = 0;
    stats_.frames_skipped = 0;
    stats_.skip_tilt = 0;
    stats_.skip_altitude = 0;
    stats_.skip_gps = 0;
    stats_.skip_image = 0;
}

bool CattleCounter::ClassAccepted(int class_id) const {
    if (options_.class_ids.empty() || class_id < 0) {
        return true;
    }
    for (size_t i = 0; i < options_.class_ids.size(); ++i) {
        if (options_.class_ids[i] == class_id) {
            return true;
        }
    }
    return false;
}

void CattleCounter::Reset() {
    tracks_.clear();
    stats_.frames_counted = 0;
    stats_.frames_skipped = 0;
    stats_.skip_tilt = 0;
    stats_.skip_altitude = 0;
    stats_.skip_gps = 0;
    stats_.skip_image = 0;
    next_id_ = 1;
    peak_in_frame_ = 0;
    survey_used_gps_ = false;
}

int CattleCounter::ConfirmedCount() const {
    int count = 0;
    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (!tracks_[i].image_only && tracks_[i].observations >= options_.min_observations) {
            count += 1;
        }
    }
    return count;
}

int CattleCounter::CandidateCount() const {
    int count = 0;
    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (!tracks_[i].image_only && tracks_[i].observations < options_.min_observations) {
            count += 1;
        }
    }
    return count;
}

int CattleCounter::PeakInFrame() const {
    return peak_in_frame_;
}

int CattleCounter::ImageTrackCount() const {
    int count = 0;
    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (tracks_[i].image_only) {
            count += 1;
        }
    }
    return count;
}

bool CattleCounter::SurveyUsedGps() const {
    return survey_used_gps_;
}

const CattleStats &CattleCounter::Stats() const {
    return stats_;
}

const std::vector<CattleFix> &CattleCounter::Fixes() const {
    return cached_fixes_;
}

void CattleCounter::RebuildFixes() {
    cached_fixes_.clear();
    for (size_t i = 0; i < tracks_.size(); ++i) {
        CattleFix fix;
        fix.id = tracks_[i].id;
        fix.lat_deg = tracks_[i].lat_deg;
        fix.lon_deg = tracks_[i].lon_deg;
        fix.observations = tracks_[i].observations;
        fix.best_score = tracks_[i].best_score;
        fix.last_time_s = tracks_[i].last_time_s;
        fix.confirmed = !tracks_[i].image_only && tracks_[i].observations >= options_.min_observations;
        fix.image_only = tracks_[i].image_only;
        cached_fixes_.push_back(fix);
    }
}

FrameResult CattleCounter::Update(const CameraPose &pose, const std::vector<CattleDetection> &detections) {
    FrameResult result;
    result.in_frame = 0;
    result.confirmed = ConfirmedCount();
    result.candidates = CandidateCount();
    result.used_gps = false;
    result.skip = CATTLE_SKIP_NONE;

    std::vector<CattleDetection> filtered;
    filtered.reserve(detections.size());
    for (size_t i = 0; i < detections.size(); ++i) {
        if (detections[i].score < options_.min_score) {
            continue;
        }
        if (!ClassAccepted(detections[i].class_id)) {
            continue;
        }
        filtered.push_back(detections[i]);
    }
    filtered = NonMaxSuppression(filtered, static_cast<float>(options_.nms_iou));

    if (!pose.gps_valid) {
        if (!options_.allow_image_tracks) {
            result.skip = CATTLE_SKIP_GPS;
            stats_.frames_skipped += 1;
            stats_.skip_gps += 1;
            result.confirmed = ConfirmedCount();
            result.candidates = CandidateCount();
            return result;
        }
        // Image-only fallback. These tracks are not survey positions.
        for (size_t i = 0; i < tracks_.size(); ++i) {
            tracks_[i].seen_this_frame = false;
        }
        for (size_t i = 0; i < filtered.size(); ++i) {
            int best = -1;
            float best_iou = 0.3f;
            for (size_t t = 0; t < tracks_.size(); ++t) {
                if (!tracks_[t].image_only || tracks_[t].seen_this_frame) {
                    continue;
                }
                float overlap = IoU(filtered[i], tracks_[t].last_box);
                if (overlap > best_iou) {
                    best_iou = overlap;
                    best = static_cast<int>(t);
                }
            }
            Track *track = 0;
            if (best >= 0) {
                track = &tracks_[best];
                track->last_box = filtered[i];
                track->observations += 1;
                track->seen_this_frame = true;
                track->last_time_s = pose.time_s;
                if (filtered[i].score > track->best_score) {
                    track->best_score = filtered[i].score;
                }
            } else {
                Track created;
                created.id = next_id_++;
                created.lat_deg = 0.0;
                created.lon_deg = 0.0;
                created.observations = 1;
                created.best_score = filtered[i].score;
                created.last_time_s = pose.time_s;
                created.image_only = true;
                created.last_box = filtered[i];
                created.seen_this_frame = true;
                tracks_.push_back(created);
                track = &tracks_.back();
            }
            FrameHit hit;
            hit.id = track->id;
            hit.confirmed = false;
            hit.cx = filtered[i].cx;
            hit.cy = filtered[i].cy;
            hit.w = filtered[i].w;
            hit.h = filtered[i].h;
            hit.score = filtered[i].score;
            result.hits.push_back(hit);
        }
        result.in_frame = static_cast<int>(filtered.size());
        if (result.in_frame > peak_in_frame_) {
            peak_in_frame_ = result.in_frame;
        }
        stats_.frames_counted += 1;
        result.confirmed = ConfirmedCount();
        result.candidates = CandidateCount();
        RebuildFixes();
        return result;
    }

    CattleSkipReason pose_skip = CATTLE_SKIP_NONE;
    double unused_lat = 0.0;
    double unused_lon = 0.0;
    double m_per_px_x = 0.0;
    double m_per_px_y = 0.0;
    if (!ProjectToGround(options_, pose, static_cast<float>(pose.width) * 0.5f,
                         static_cast<float>(pose.height) * 0.5f, &unused_lat, &unused_lon,
                         &m_per_px_x, &m_per_px_y, &pose_skip)) {
        result.skip = pose_skip;
        stats_.frames_skipped += 1;
        if (pose_skip == CATTLE_SKIP_TILT) {
            stats_.skip_tilt += 1;
        } else if (pose_skip == CATTLE_SKIP_ALTITUDE) {
            stats_.skip_altitude += 1;
        } else if (pose_skip == CATTLE_SKIP_GPS) {
            stats_.skip_gps += 1;
        } else {
            stats_.skip_image += 1;
        }
        result.confirmed = ConfirmedCount();
        result.candidates = CandidateCount();
        return result;
    }

    std::vector<ProjectedDet> projected;
    for (size_t i = 0; i < filtered.size(); ++i) {
        double length_m = std::max(filtered[i].w * static_cast<float>(m_per_px_x),
                                   filtered[i].h * static_cast<float>(m_per_px_y));
        if (length_m < options_.min_length_m || length_m > options_.max_length_m) {
            continue;
        }
        ProjectedDet item;
        item.box = filtered[i];
        item.length_m = length_m;
        CattleSkipReason one_skip = CATTLE_SKIP_NONE;
        if (!ProjectToGround(options_, pose, filtered[i].cx, filtered[i].cy, &item.lat_deg, &item.lon_deg,
                             0, 0, &one_skip)) {
            continue;
        }
        bool duplicate = false;
        for (size_t k = 0; k < projected.size(); ++k) {
            if (MetersBetween(projected[k].lat_deg, projected[k].lon_deg, item.lat_deg, item.lon_deg) <
                options_.dedup_radius_m) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            projected.push_back(item);
        }
    }

    std::sort(projected.begin(), projected.end(), [](const ProjectedDet &a, const ProjectedDet &b) {
        return a.box.score > b.box.score;
    });

    std::vector<char> used(tracks_.size(), 0);
    for (size_t i = 0; i < projected.size(); ++i) {
        int best = -1;
        double best_dist = 0.0;
        for (size_t t = 0; t < tracks_.size(); ++t) {
            if (used[t] || tracks_[t].image_only) {
                continue;
            }
            double dt = pose.time_s - tracks_[t].last_time_s;
            if (dt < 0.0) {
                dt = 0.0;
            }
            double extra = options_.max_animal_speed_m_s * dt;
            if (extra > options_.max_motion_m) {
                extra = options_.max_motion_m;
            }
            double radius = options_.match_radius_m + extra;
            double dist = MetersBetween(tracks_[t].lat_deg, tracks_[t].lon_deg, projected[i].lat_deg,
                                        projected[i].lon_deg);
            if (dist <= radius && (best < 0 || dist < best_dist)) {
                best = static_cast<int>(t);
                best_dist = dist;
            }
        }
        Track *track = 0;
        if (best >= 0) {
            used[best] = 1;
            track = &tracks_[best];
            track->lat_deg = track->lat_deg * 0.65 + projected[i].lat_deg * 0.35;
            track->lon_deg = track->lon_deg * 0.65 + projected[i].lon_deg * 0.35;
            track->observations += 1;
            track->last_time_s = pose.time_s;
            track->last_box = projected[i].box;
            if (projected[i].box.score > track->best_score) {
                track->best_score = projected[i].box.score;
            }
        } else {
            Track created;
            created.id = next_id_++;
            created.lat_deg = projected[i].lat_deg;
            created.lon_deg = projected[i].lon_deg;
            created.observations = 1;
            created.best_score = projected[i].box.score;
            created.last_time_s = pose.time_s;
            created.image_only = false;
            created.last_box = projected[i].box;
            created.seen_this_frame = true;
            tracks_.push_back(created);
            used.push_back(1);
            track = &tracks_.back();
        }
        FrameHit hit;
        hit.id = track->id;
        hit.confirmed = track->observations >= options_.min_observations;
        hit.cx = projected[i].box.cx;
        hit.cy = projected[i].box.cy;
        hit.w = projected[i].box.w;
        hit.h = projected[i].box.h;
        hit.score = projected[i].box.score;
        result.hits.push_back(hit);
    }

    survey_used_gps_ = true;
    result.used_gps = true;
    result.in_frame = static_cast<int>(projected.size());
    if (result.in_frame > peak_in_frame_) {
        peak_in_frame_ = result.in_frame;
    }
    stats_.frames_counted += 1;
    result.confirmed = ConfirmedCount();
    result.candidates = CandidateCount();
    result.skip = CATTLE_SKIP_NONE;
    RebuildFixes();
    return result;
}

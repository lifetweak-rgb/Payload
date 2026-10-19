#ifndef CATTLE_COUNTER_HPP
#define CATTLE_COUNTER_HPP

#include <string>
#include <vector>

// Counts cattle in a downward-looking video without counting the same animal
// again as the aircraft flies. Each detection is projected onto the ground
// with the wide-camera field of view, altitude, and gimbal attitude, then
// matched to an animal already seen nearby.

enum CattleSkipReason {
    CATTLE_SKIP_NONE = 0,
    CATTLE_SKIP_TILT,
    CATTLE_SKIP_ALTITUDE,
    CATTLE_SKIP_GPS,
    CATTLE_SKIP_IMAGE
};

struct CattleOptions {
    double hfov_deg;
    double dfov_deg;
    double sensor_width;
    double sensor_height;
    double nadir_pitch_deg;
    double max_tilt_from_nadir_deg;
    double match_radius_m;
    double max_animal_speed_m_s;
    double max_motion_m;
    double dedup_radius_m;
    double min_length_m;
    double max_length_m;
    double min_score;
    double min_alt_m;
    double max_alt_m;
    int min_observations;
    double nms_iou;
    bool allow_image_tracks;
    std::vector<int> class_ids;
};

CattleOptions DefaultCattleOptions();

struct CattleDetection {
    float cx;
    float cy;
    float w;
    float h;
    float score;
    int class_id;
};

struct CameraPose {
    double time_s;
    double lat_deg;
    double lon_deg;
    double alt_agl_m;
    double gimbal_pitch_deg;
    double gimbal_yaw_deg;
    int width;
    int height;
    bool gps_valid;
};

struct CattleFix {
    int id;
    double lat_deg;
    double lon_deg;
    int observations;
    float best_score;
    double last_time_s;
    bool confirmed;
    bool image_only;
};

struct FrameHit {
    int id;
    bool confirmed;
    float cx;
    float cy;
    float w;
    float h;
    float score;
};

struct FrameResult {
    int in_frame;
    int confirmed;
    int candidates;
    bool used_gps;
    CattleSkipReason skip;
    std::vector<FrameHit> hits;
};

struct CattleStats {
    int frames_counted;
    int frames_skipped;
    int skip_tilt;
    int skip_altitude;
    int skip_gps;
    int skip_image;
};

// YOLOv8-style tensor. channel_major layout is data[attr * num_boxes + box].
// box_major layout is data[box * num_attrs + attr]. The first four attributes
// are cx, cy, w, h in input-image pixels (or normalized 0-1). Remaining
// attributes are class scores.
std::vector<CattleDetection> ParseYoloV8(const float *data,
                                         int num_attrs,
                                         int num_boxes,
                                         bool channel_major,
                                         int input_w,
                                         int input_h,
                                         int frame_w,
                                         int frame_h,
                                         float conf_thres,
                                         const std::vector<int> &class_ids);

std::vector<CattleDetection> NonMaxSuppression(const std::vector<CattleDetection> &boxes,
                                               float iou_thres);

// Letterbox used by the aerial YOLOv8 model. The network input is square;
// the source is resized with unchanged aspect ratio and centered on a pad.
struct LetterboxFit {
    float scale;
    float pad_x;
    float pad_y;
    int resized_w;
    int resized_h;
};

LetterboxFit FitLetterbox(int src_w, int src_h, int dst);

// Maps a box from letterboxed network pixels back onto the source image.
// origin_x and origin_y are the source-image origin of this tile.
void MapLetterboxDetection(CattleDetection *detection,
                           const LetterboxFit &fit,
                           float origin_x,
                           float origin_y);

// Tile origins that cover [0, length). The last origin is length - tile when
// the span is longer than one tile, so the far edge is included.
std::vector<int> TileStarts(int length, int tile, int stride);

class CattleCounter {
 public:
    explicit CattleCounter(const CattleOptions &options);

    FrameResult Update(const CameraPose &pose, const std::vector<CattleDetection> &detections);

    int ConfirmedCount() const;
    int CandidateCount() const;
    int PeakInFrame() const;
    int ImageTrackCount() const;
    bool SurveyUsedGps() const;
    const CattleStats &Stats() const;
    const std::vector<CattleFix> &Fixes() const;
    void Reset();

    static double HorizontalFovDeg(const CattleOptions &options);

    // Projects a pixel onto the local ground plane. On success, meters_per_px_*
    // describe the ground sample distance at the principal point.
    static bool ProjectToGround(const CattleOptions &options,
                                const CameraPose &pose,
                                float cx,
                                float cy,
                                double *lat_deg,
                                double *lon_deg,
                                double *meters_per_px_x,
                                double *meters_per_px_y,
                                CattleSkipReason *skip);

 private:
    struct Track {
        int id;
        double lat_deg;
        double lon_deg;
        int observations;
        float best_score;
        double last_time_s;
        bool image_only;
        CattleDetection last_box;
        bool seen_this_frame;
    };

    CattleOptions options_;
    std::vector<Track> tracks_;
    CattleStats stats_;
    int next_id_;
    int peak_in_frame_;
    bool survey_used_gps_;

    static double MetersBetween(double lat1, double lon1, double lat2, double lon2);
    bool ClassAccepted(int class_id) const;
    void RebuildFixes();

    std::vector<CattleFix> cached_fixes_;
};

const char *CattleSkipReasonText(CattleSkipReason reason);

#endif

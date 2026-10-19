#ifndef CATTLE_CONFIG_HPP
#define CATTLE_CONFIG_HPP

#include "cattle_counter.hpp"

#include <string>

struct CattleAppConfig {
    CattleOptions counter;
    std::string python;
    std::string email_script;
    std::string output_dir;
    std::string onnx_model;
    int onnx_input;
    bool email_on_exit;
    int camera_source;
    int camera_position;
    bool ground_altitude_set;
    double ground_altitude_m;
    std::string config_path;
};

CattleAppConfig DefaultCattleAppConfig();

// Reads the flat JSON object used by both the counter and send_report.py.
// Unknown keys are ignored so SMTP settings can live in the same file.
bool LoadCattleAppConfig(const std::string &path, CattleAppConfig *config, std::string *error);

// $CATTLE_CONFIG, then ./cattle/config.json, then /open_app/cattle_config.json,
// then ./cattle/config.example.json.
std::string FindCattleConfigPath();

#endif

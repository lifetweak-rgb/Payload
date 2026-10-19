#include "cattle_config.hpp"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <utility>

namespace {

enum ValueType { VALUE_NONE, VALUE_NUMBER, VALUE_STRING, VALUE_BOOL, VALUE_ARRAY, VALUE_NULL };

struct JsonValue {
    ValueType type;
    double number;
    std::string text;
    bool flag;
    std::vector<double> numbers;

    JsonValue() : type(VALUE_NONE), number(0.0), flag(false) {}
};

class Parser {
 public:
    explicit Parser(const std::string &text) : text_(text), index_(0) {}

    bool ParseObject(std::vector<std::pair<std::string, JsonValue> > *pairs, std::string *error) {
        Skip();
        if (!Consume('{')) {
            return Fail("expected '{' at the start of the config", error);
        }
        Skip();
        if (Consume('}')) {
            return true;
        }
        while (index_ < text_.size()) {
            Skip();
            std::string key;
            if (!ParseString(&key, error)) {
                return false;
            }
            Skip();
            if (!Consume(':')) {
                return Fail("expected ':' after a config key", error);
            }
            JsonValue value;
            if (!ParseValue(&value, error)) {
                return false;
            }
            pairs->push_back(std::make_pair(key, value));
            Skip();
            if (Consume('}')) {
                return true;
            }
            if (!Consume(',')) {
                return Fail("expected ',' between config fields", error);
            }
        }
        return Fail("config object was not closed", error);
    }

 private:
    std::string text_;
    size_t index_;

    bool Fail(const std::string &message, std::string *error) const {
        if (error) {
            std::ostringstream out;
            out << message << " at character " << index_;
            *error = out.str();
        }
        return false;
    }

    void Skip() {
        while (index_ < text_.size()) {
            char c = text_[index_];
            if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
                index_ += 1;
            } else {
                break;
            }
        }
    }

    bool Consume(char expected) {
        Skip();
        if (index_ < text_.size() && text_[index_] == expected) {
            index_ += 1;
            return true;
        }
        return false;
    }

    bool ParseValue(JsonValue *value, std::string *error) {
        Skip();
        if (index_ >= text_.size()) {
            return Fail("expected a value", error);
        }
        char c = text_[index_];
        if (c == '"') {
            value->type = VALUE_STRING;
            return ParseString(&value->text, error);
        }
        if (c == '[') {
            value->type = VALUE_ARRAY;
            return ParseArray(&value->numbers, error);
        }
        if (text_.compare(index_, 4, "true") == 0) {
            value->type = VALUE_BOOL;
            value->flag = true;
            index_ += 4;
            return true;
        }
        if (text_.compare(index_, 5, "false") == 0) {
            value->type = VALUE_BOOL;
            value->flag = false;
            index_ += 5;
            return true;
        }
        if (text_.compare(index_, 4, "null") == 0) {
            value->type = VALUE_NULL;
            index_ += 4;
            return true;
        }
        return ParseNumber(&value->number, error) && (value->type = VALUE_NUMBER, true);
    }

    bool ParseArray(std::vector<double> *numbers, std::string *error) {
        if (!Consume('[')) {
            return Fail("expected '['", error);
        }
        Skip();
        if (Consume(']')) {
            return true;
        }
        while (index_ < text_.size()) {
            double number = 0.0;
            if (!ParseNumber(&number, error)) {
                return false;
            }
            numbers->push_back(number);
            Skip();
            if (Consume(']')) {
                return true;
            }
            if (!Consume(',')) {
                return Fail("expected ',' in a number list", error);
            }
        }
        return Fail("number list was not closed", error);
    }

    bool ParseNumber(double *number, std::string *error) {
        Skip();
        size_t start = index_;
        if (index_ < text_.size() && (text_[index_] == '-' || text_[index_] == '+')) {
            index_ += 1;
        }
        bool saw_digit = false;
        while (index_ < text_.size() && text_[index_] >= '0' && text_[index_] <= '9') {
            saw_digit = true;
            index_ += 1;
        }
        if (index_ < text_.size() && text_[index_] == '.') {
            index_ += 1;
            while (index_ < text_.size() && text_[index_] >= '0' && text_[index_] <= '9') {
                saw_digit = true;
                index_ += 1;
            }
        }
        if (index_ < text_.size() && (text_[index_] == 'e' || text_[index_] == 'E')) {
            index_ += 1;
            if (index_ < text_.size() && (text_[index_] == '-' || text_[index_] == '+')) {
                index_ += 1;
            }
            while (index_ < text_.size() && text_[index_] >= '0' && text_[index_] <= '9') {
                saw_digit = true;
                index_ += 1;
            }
        }
        if (!saw_digit) {
            return Fail("expected a number", error);
        }
        *number = std::atof(text_.c_str() + start);
        return true;
    }

    bool ParseString(std::string *text, std::string *error) {
        Skip();
        if (!Consume('"')) {
            return Fail("expected a string", error);
        }
        text->clear();
        while (index_ < text_.size()) {
            char c = text_[index_];
            index_ += 1;
            if (c == '"') {
                return true;
            }
            if (c == '\\') {
                if (index_ >= text_.size()) {
                    return Fail("string ended in a backslash", error);
                }
                char esc = text_[index_];
                index_ += 1;
                if (esc == '"' || esc == '\\' || esc == '/') {
                    text->push_back(esc);
                } else if (esc == 'n') {
                    text->push_back('\n');
                } else if (esc == 't') {
                    text->push_back('\t');
                } else {
                    return Fail("unsupported string escape", error);
                }
            } else {
                text->push_back(c);
            }
        }
        return Fail("string was not closed", error);
    }
};

bool ApplyKey(CattleAppConfig *config, const std::string &key, const JsonValue &value, std::string *error) {
    CattleOptions &counter = config->counter;
    if (key == "hfov_deg" && value.type == VALUE_NUMBER) {
        counter.hfov_deg = value.number;
    } else if (key == "dfov_deg" && value.type == VALUE_NUMBER) {
        counter.dfov_deg = value.number;
    } else if (key == "sensor_width" && value.type == VALUE_NUMBER) {
        counter.sensor_width = value.number;
    } else if (key == "sensor_height" && value.type == VALUE_NUMBER) {
        counter.sensor_height = value.number;
    } else if (key == "nadir_pitch_deg" && value.type == VALUE_NUMBER) {
        counter.nadir_pitch_deg = value.number;
    } else if (key == "max_tilt_deg" && value.type == VALUE_NUMBER) {
        counter.max_tilt_from_nadir_deg = value.number;
    } else if (key == "match_radius_m" && value.type == VALUE_NUMBER) {
        counter.match_radius_m = value.number;
    } else if (key == "max_animal_speed_m_s" && value.type == VALUE_NUMBER) {
        counter.max_animal_speed_m_s = value.number;
    } else if (key == "max_motion_m" && value.type == VALUE_NUMBER) {
        counter.max_motion_m = value.number;
    } else if (key == "dedup_radius_m" && value.type == VALUE_NUMBER) {
        counter.dedup_radius_m = value.number;
    } else if (key == "min_length_m" && value.type == VALUE_NUMBER) {
        counter.min_length_m = value.number;
    } else if (key == "max_length_m" && value.type == VALUE_NUMBER) {
        counter.max_length_m = value.number;
    } else if (key == "min_score" && value.type == VALUE_NUMBER) {
        counter.min_score = value.number;
    } else if (key == "min_alt_m" && value.type == VALUE_NUMBER) {
        counter.min_alt_m = value.number;
    } else if (key == "max_alt_m" && value.type == VALUE_NUMBER) {
        counter.max_alt_m = value.number;
    } else if (key == "min_observations" && value.type == VALUE_NUMBER) {
        counter.min_observations = static_cast<int>(value.number);
    } else if (key == "nms_iou" && value.type == VALUE_NUMBER) {
        counter.nms_iou = value.number;
    } else if (key == "allow_image_tracks" && value.type == VALUE_BOOL) {
        counter.allow_image_tracks = value.flag;
    } else if (key == "class_ids" && value.type == VALUE_ARRAY) {
        counter.class_ids.clear();
        for (size_t i = 0; i < value.numbers.size(); ++i) {
            counter.class_ids.push_back(static_cast<int>(value.numbers[i]));
        }
    } else if (key == "python" && value.type == VALUE_STRING) {
        config->python = value.text;
    } else if (key == "email_script" && value.type == VALUE_STRING) {
        config->email_script = value.text;
    } else if (key == "output_dir" && value.type == VALUE_STRING) {
        config->output_dir = value.text;
    } else if (key == "onnx_model" && value.type == VALUE_STRING) {
        config->onnx_model = value.text;
    } else if (key == "onnx_input" && value.type == VALUE_NUMBER) {
        config->onnx_input = static_cast<int>(value.number);
    } else if (key == "email_on_exit" && value.type == VALUE_BOOL) {
        config->email_on_exit = value.flag;
    } else if (key == "camera_source" && value.type == VALUE_NUMBER) {
        config->camera_source = static_cast<int>(value.number);
    } else if (key == "camera_position" && value.type == VALUE_NUMBER) {
        config->camera_position = static_cast<int>(value.number);
    } else if (key == "ground_altitude_m" && value.type == VALUE_NUMBER) {
        config->ground_altitude_set = true;
        config->ground_altitude_m = value.number;
    } else if (value.type == VALUE_NONE) {
        if (error) {
            *error = "unsupported value for " + key;
        }
        return false;
    }
    return true;
}

bool FileExists(const std::string &path) {
    std::ifstream input(path.c_str());
    return input.good();
}

std::string ParentDirectory(const std::string &path) {
    size_t slash = path.find_last_of('/');
    if (slash == std::string::npos) {
        return ".";
    }
    if (slash == 0) {
        return "/";
    }
    return path.substr(0, slash);
}

std::string JoinRelative(const std::string &dir, const std::string &rel) {
    std::string clean = rel;
    if (clean.size() >= 2 && clean[0] == '.' && clean[1] == '/') {
        clean = clean.substr(2);
    }
    if (dir.empty() || dir == ".") {
        return clean;
    }
    if (dir == "/") {
        return "/" + clean;
    }
    return dir + "/" + clean;
}

void ResolveOnnxModel(CattleAppConfig *config) {
    if (!config || config->onnx_model.empty() || config->onnx_model[0] == '/') {
        return;
    }
    config->onnx_model = JoinRelative(ParentDirectory(config->config_path), config->onnx_model);
}

}  // namespace

CattleAppConfig DefaultCattleAppConfig() {
    CattleAppConfig config;
    config.counter = DefaultCattleOptions();
    config.python = "python3";
    config.email_script = "cattle/send_report.py";
    config.output_dir = "data/cattle";
    config.onnx_input = 640;
    config.email_on_exit = true;
    config.camera_source = 1;
    config.camera_position = 1;
    config.ground_altitude_set = false;
    config.ground_altitude_m = 0.0;
    return config;
}

bool LoadCattleAppConfig(const std::string &path, CattleAppConfig *config, std::string *error) {
    if (!config) {
        if (error) {
            *error = "missing config output";
        }
        return false;
    }
    *config = DefaultCattleAppConfig();
    config->config_path = path;
    std::ifstream input(path.c_str());
    if (!input) {
        if (error) {
            *error = "could not open " + path;
        }
        return false;
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    Parser parser(buffer.str());
    std::vector<std::pair<std::string, JsonValue> > pairs;
    if (!parser.ParseObject(&pairs, error)) {
        return false;
    }
    for (size_t i = 0; i < pairs.size(); ++i) {
        if (!ApplyKey(config, pairs[i].first, pairs[i].second, error)) {
            return false;
        }
    }
    ResolveOnnxModel(config);
    return true;
}

std::string FindCattleConfigPath() {
    const char *from_env = std::getenv("CATTLE_CONFIG");
    if (from_env && from_env[0] != '\0' && FileExists(from_env)) {
        return from_env;
    }
    const char *candidates[] = {
        "cattle/config.json",
        "/open_app/cattle_config.json",
        "cattle/config.example.json",
    };
    for (size_t i = 0; i < 3; ++i) {
        if (FileExists(candidates[i])) {
            return candidates[i];
        }
    }
    return "cattle/config.example.json";
}

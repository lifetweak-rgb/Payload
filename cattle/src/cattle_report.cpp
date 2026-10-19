#include "cattle_report.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace {

std::string FormatFixed(double value, int digits) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
    return buffer;
}

}  // namespace

int CattleHeadlineCount(const CattleCounter &counter) {
    if (counter.SurveyUsedGps()) {
        return counter.ConfirmedCount();
    }
    return counter.ImageTrackCount();
}

std::string BuildCattleReport(const CattleCounter &counter,
                              const std::string &aircraft,
                              const std::string &when_utc) {
    const CattleStats &stats = counter.Stats();
    int headline = CattleHeadlineCount(counter);
    std::ostringstream out;
    out << "Matrice 4T cattle count\n";
    out << "Aircraft: " << aircraft << "\n";
    out << "When: " << when_utc << "\n";
    out << "Cattle count: " << headline << "\n";
    if (counter.SurveyUsedGps()) {
        out << "Confirmed cattle: " << counter.ConfirmedCount() << "\n";
        out << "Unconfirmed sightings: " << counter.CandidateCount() << "\n";
        out << "Peak cattle in one frame: " << counter.PeakInFrame() << "\n";
        out << "\n";
        out << "Each confirmed animal was seen at least the configured number of\n";
        out << "times at the same ground position, so it is not counted again as\n";
        out << "the aircraft moves.\n";
    } else if (counter.ImageTrackCount() > 0) {
        out << "GPS survey: not available\n";
        out << "Image-track cattle: " << counter.ImageTrackCount() << "\n";
        out << "Peak cattle in one frame: " << counter.PeakInFrame() << "\n";
        out << "\n";
        out << "This number follows animals only while they stay in the picture.\n";
        out << "The same animal is counted again after it leaves and comes back.\n";
        out << "Mark the ground altitude and fly with a GPS fix for a pasture total.\n";
    } else {
        out << "Confirmed cattle: 0\n";
        out << "GPS survey: not available\n";
        out << "\n";
        out << "Nothing was counted. Mark the ground altitude on the ground, point\n";
        out << "the wide camera straight down, and fly with a GPS fix.\n";
    }
    out << "\n";
    out << "Frames counted: " << stats.frames_counted << "\n";
    out << "Frames skipped: " << stats.frames_skipped << "\n";
    out << "Skipped for tilt: " << stats.skip_tilt << "\n";
    out << "Skipped for altitude: " << stats.skip_altitude << "\n";
    out << "Skipped for GPS: " << stats.skip_gps << "\n";
    out << "\n";
    out << "Animals:\n";
    const std::vector<CattleFix> &fixes = counter.Fixes();
    if (fixes.empty()) {
        out << "(none)\n";
    }
    for (size_t i = 0; i < fixes.size(); ++i) {
        const CattleFix &fix = fixes[i];
        out << fix.id << ". ";
        if (fix.image_only) {
            out << "image-only";
        } else if (fix.confirmed) {
            out << "confirmed";
        } else {
            out << "unconfirmed";
        }
        out << " lat=" << FormatFixed(fix.lat_deg, 7);
        out << " lon=" << FormatFixed(fix.lon_deg, 7);
        out << " observations=" << fix.observations;
        out << " score=" << FormatFixed(fix.best_score, 2);
        out << "\n";
    }
    out << "\n";
    out << "This is an automated estimate from the wide camera. Animals under\n";
    out << "trees, standing closer together than the match radius, or outside\n";
    out << "the downward view can be missed or merged.\n";
    return out.str();
}

bool WriteTextFile(const std::string &path, const std::string &text, std::string *error) {
    std::ofstream output(path.c_str(), std::ios::out | std::ios::binary);
    if (!output) {
        if (error) {
            *error = "could not write " + path;
        }
        return false;
    }
    output << text;
    if (!output.good()) {
        if (error) {
            *error = "failed while writing " + path;
        }
        return false;
    }
    return true;
}

std::string ShellQuote(const std::string &text) {
    std::string out = "'";
    for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\'') {
            out += "'\\''";
        } else {
            out += text[i];
        }
    }
    out += "'";
    return out;
}

std::string BuildEmailCommand(const CattleEmailRequest &request) {
    std::ostringstream cmd;
    std::string python = request.python.empty() ? "python3" : request.python;
    cmd << ShellQuote(python) << " " << ShellQuote(request.script);
    cmd << " --config " << ShellQuote(request.config_path);
    cmd << " --body " << ShellQuote(request.body_path);
    cmd << " --subject " << ShellQuote(request.subject);
    if (!request.attachment.empty()) {
        cmd << " --attach " << ShellQuote(request.attachment);
    }
    return cmd.str();
}

int SendCattleEmail(const CattleEmailRequest &request, std::string *error) {
    std::string command = BuildEmailCommand(request);
    int status = std::system(command.c_str());
    if (status != 0 && error) {
        std::ostringstream out;
        out << "email command failed with status " << status << ": " << command;
        *error = out.str();
    }
    return status;
}

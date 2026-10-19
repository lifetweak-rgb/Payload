#ifndef CATTLE_REPORT_HPP
#define CATTLE_REPORT_HPP

#include "cattle_counter.hpp"

#include <string>

struct CattleEmailRequest {
    std::string python;
    std::string script;
    std::string config_path;
    std::string body_path;
    std::string subject;
    std::string attachment;
};

// Number the email subject should advertise. GPS surveys use the confirmed
// unique count. With no GPS, the image-track count is used only when that
// mode is on; otherwise the headline stays 0.
int CattleHeadlineCount(const CattleCounter &counter);

std::string BuildCattleReport(const CattleCounter &counter,
                              const std::string &aircraft,
                              const std::string &when_utc);

bool WriteTextFile(const std::string &path, const std::string &text, std::string *error);

std::string ShellQuote(const std::string &text);

// argv-style command. Empty attachment is omitted. The command is not executed.
std::string BuildEmailCommand(const CattleEmailRequest &request);

// Runs BuildEmailCommand. Returns the process status from std::system.
int SendCattleEmail(const CattleEmailRequest &request, std::string *error);

#endif

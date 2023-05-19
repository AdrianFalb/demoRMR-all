#ifndef UTILITIES_H
#define UTILITIES_H

#include <regex>

static bool validateIp(const std::string ip) {

    std::regex ipPattern(R"(^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$)");
    std::smatch matches;

    if (std::regex_match(ip, matches, ipPattern)) {
        for (size_t i = 1; i <= 4; ++i) {
            int octet = std::stoi(matches[i]);
            if (octet < 0 || octet > 255) {
                return false;
            }
        }
        return true;
    }

    return false;
}

#endif // UTILITIES_H

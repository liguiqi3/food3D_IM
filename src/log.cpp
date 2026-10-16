// Conan::ImportStart
#include "log.hpp"
#include <string>
// Conan::ImportEnd

#ifndef __ARM_EABI__
#include <time.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#endif



#ifdef __ARM_EABI__

// Bare-metal has no file system or console; logging degrades to a no-op.

void log_set_level(LogLevel /*level*/) {}

LogLevel log_get_level() { return LogLevel::kOff; }

void log_set_console(bool /*enabled*/) {}

bool log_get_console() { return false; }

void log_set_file(const std::string& /*path*/, LogFileMode /*mode*/) {}

void log_close_file() {}

void log_write(LogLevel /*level*/, const std::string& /*message*/) {}

void log_trace(const std::string& /*message*/) {}

void log_debug(const std::string& /*message*/) {}

void log_info(const std::string& /*message*/) {}

void log_warning(const std::string& /*message*/) {}

void log_error(const std::string& /*message*/) {}

#else



namespace {

std::mutex g_log_mutex;
LogLevel g_log_level = LogLevel::kInfo;
bool g_log_console = false;
std::ofstream g_log_file;

const char* level_name(LogLevel level) {
    switch (level) {
    case LogLevel::kTrace:
        return "TRACE";
    case LogLevel::kDebug:
        return "DEBUG";
    case LogLevel::kInfo:
        return "INFO";
    case LogLevel::kWarning:
        return "WARN";
    case LogLevel::kError:
        return "ERROR";
    default:
        return "OFF";
    }
}

std::string timestamp() {
    const time_t now = time(nullptr);
    tm t{};
    if (localtime_r(&now, &t) == nullptr) {
        return "0000-00-00 00:00:00";
    }
    std::ostringstream stream;
    stream << std::put_time(&t, "%Y-%m-%d %H:%M:%S");
    return stream.str();
}

} // namespace



void log_set_level(LogLevel level) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_level = level;
}



LogLevel log_get_level() {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    return g_log_level;
}



void log_set_console(bool enabled) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_console = enabled;
}



bool log_get_console() {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    return g_log_console;
}



void log_set_file(const std::string& path, LogFileMode mode) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    if (g_log_file.is_open()) {
        g_log_file.close();
    }
    if (path.empty()) {
        return;
    }
    g_log_file.open(path, mode == LogFileMode::kAppend ? std::ios::app : std::ios::trunc);
}



void log_close_file() {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    if (g_log_file.is_open()) {
        g_log_file.close();
    }
}



void log_write(LogLevel level, const std::string& message) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    if (static_cast<int>(level) < static_cast<int>(g_log_level)) {
        return;
    }
    const std::string line = "[" + timestamp() + "] [" + level_name(level) + "] " + message;
    if (g_log_console) {
        if (level == LogLevel::kWarning || level == LogLevel::kError) {
            std::cerr << line << '\n';
        } else {
            std::cout << line << '\n';
        }
    }
    if (g_log_file.is_open()) {
        g_log_file << line << '\n';
        g_log_file.flush();
    }
}



void log_trace(const std::string& message) { log_write(LogLevel::kTrace, message); }



void log_debug(const std::string& message) { log_write(LogLevel::kDebug, message); }



void log_info(const std::string& message) { log_write(LogLevel::kInfo, message); }



void log_warning(const std::string& message) { log_write(LogLevel::kWarning, message); }



void log_error(const std::string& message) { log_write(LogLevel::kError, message); }



#endif // __ARM_EABI__

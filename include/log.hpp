// Conan::ImportStart
#pragma once
#include <cstdint>
#include <string>
// Conan::ImportEnd



/**
 * @brief [en] Logging severity levels.
 * @brief [zh] 日志严重级别。
 * @exporter
 */
enum class LogLevel : std::uint8_t {
    kTrace = 0,
    kDebug = 1,
    kInfo = 2,
    kWarning = 3,
    kError = 4,
    kOff = 5,
};



/**
 * @brief [en] Write mode of the log file sink.
 * @brief [zh] 日志文件 sink 的写入模式。
 * @exporter
 */
enum class LogFileMode : std::uint8_t {
    kAppend = 0,
    kTruncate = 1,
};



/**
 * @brief [en] Sets the minimum emitted log level; messages below it are dropped.
 * @brief [zh] 设置要输出的最小日志级别；低于该级别的消息被丢弃。
 * @param level [en] Minimum level to emit.
 * @param level [zh] 要输出的最低级别。
 * @exporter
 */
void log_set_level(LogLevel level);



/**
 * @brief [en] Returns the current minimum emitted log level.
 * @brief [zh] 返回当前要输出的最低日志级别。
 * @exporter
 */
LogLevel log_get_level();



/**
 * @brief [en] Enables or disables console output.
 * @brief [zh] 启用或禁用控制台输出。
 * @param enabled [en] True to print to the console.
 * @param enabled [zh] 为真时打印到控制台。
 * @exporter
 */
void log_set_console(bool enabled);



/**
 * @brief [en] Returns whether console output is enabled.
 * @brief [zh] 返回控制台输出是否启用。
 * @exporter
 */
bool log_get_console();



/**
 * @brief [en] Opens a log file; an empty path closes the current file.
 * @brief [zh] 打开日志文件；空路径则关闭当前文件。
 * @param path [en] Output text file path.
 * @param path [zh] 输出文本文件路径。
 * @param mode [en] Append to or truncate the file.
 * @param mode [zh] 追加或覆盖文件。
 * @exporter
 */
void log_set_file(const std::string& path, LogFileMode mode = LogFileMode::kAppend);



/**
 * @brief [en] Closes the log file if it is open.
 * @brief [zh] 若日志文件已打开则关闭它。
 * @exporter
 */
void log_close_file();



/**
 * @brief [en] Writes one log line when the level is at or above the configured minimum.
 * @brief [zh] 当级别不低于配置的最小级别时写入一行日志。
 * @param level [en] Severity of the message.
 * @param level [zh] 消息级别。
 * @param message [en] Message text.
 * @param message [zh] 消息文本。
 * @exporter
 */
void log_write(LogLevel level, const std::string& message);



/**
 * @brief [en] Writes a trace-level message.
 * @brief [zh] 写入 trace 级别消息。
 * @exporter
 */
void log_trace(const std::string& message);



/**
 * @brief [en] Writes a debug-level message.
 * @brief [zh] 写入 debug 级别消息。
 * @exporter
 */
void log_debug(const std::string& message);



/**
 * @brief [en] Writes an info-level message.
 * @brief [zh] 写入 info 级别消息。
 * @exporter
 */
void log_info(const std::string& message);



/**
 * @brief [en] Writes a warning-level message.
 * @brief [zh] 写入 warning 级别消息。
 * @exporter
 */
void log_warning(const std::string& message);



/**
 * @brief [en] Writes an error-level message.
 * @brief [zh] 写入 error 级别消息。
 * @exporter
 */
void log_error(const std::string& message);

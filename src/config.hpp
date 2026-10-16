// Conan::ImportStart
#pragma once
#include <string>
#include "types.hpp"
// Conan::ImportEnd



/**
 * @brief [en] Loads a `MeasurementConfig` from a JSON file.
 * @brief [zh] 从 JSON 文件载入 `MeasurementConfig`。
 *
 * @param path [en] Path to the JSON config file.
 * @param path [zh] JSON 配置文件路径。
 * @param out [en] Config filled from the file. Missing fields keep their
 *     existing values, so callers may start from a default-initialized config
 *     and override only the keys present in the file.
 * @param out [zh] 从文件填充的配置；缺失字段保留原值，因此调用方可用默认配置
 *     初始化，仅覆盖文件中出现的键。
 * @return [en] True when the file was parsed and all present fields were valid.
 * @return [zh] 文件解析成功且所有出现的字段均有效时为真。
 *
 * @details [en] The JSON schema mirrors `MeasurementConfig` field names:
 *     - `"input_unit"`: `"meter"` or `"millimeter"`
 *     - `"roi"`: bounds object to enable cropping, or null to disable it
 *     - `"selected_labels"`: array of integers; empty selects all eligible components
 *     - `"middle_cloud_dir"`: nonempty directory enables intermediate point-cloud dumps
 *     - all other fields are scalars matching the struct member names.
 *     Unknown or removed keys are rejected.
 * @details [zh] JSON 结构与 `MeasurementConfig` 字段同名：
 *     - `"input_unit"`：`"meter"` 或 `"millimeter"`
 *     - `"roi"`：边界对象启用裁剪，null 禁用裁剪
 *     - `"selected_labels"`：整数数组；空数组选择所有符合条件的连通块
 *     - `"middle_cloud_dir"`：非空目录启用中间点云保存
 *     - 其余字段为与结构体成员同名的标量；未知或已删除的键会报错。

 */
bool load_config_from_json(const std::string& path, MeasurementConfig& out);



/**
 * @brief [en] Saves a `MeasurementConfig` to a JSON file.
 * @brief [zh] 将 `MeasurementConfig` 保存到 JSON 文件。
 *
 * @param cfg [en] Config to serialize.
 * @param cfg [zh] 要序列化的配置。
 * @param path [en] Output JSON file path.
 * @param path [zh] 输出的 JSON 文件路径。
 * @return [en] True when the file was written successfully.
 * @return [zh] 文件成功写入时为真。
 *
 * @details [en] The emitted JSON mirrors the same schema `load_config_from_json`
 *     reads (field names match `MeasurementConfig` members; enums are written
 *     as their string forms).
 * @details [zh] 输出的 JSON 与 `load_config_from_json` 读取的 schema 一致
 *     （字段名与 `MeasurementConfig` 成员同名；枚举以字符串形式写出）。

 */
bool save_config_to_json(const MeasurementConfig& cfg, const std::string& path);

# food3d_im 上板性能 Benchmark 指标建议

## 目的与当前基线

本文定义 `food3d_im` 在 Linux A 核板端的性能采集口径，目标是让每次 CI、Conan 包和上板运行都能回答：算法是否正确、快了还是慢了、占用多少运行内存，以及部署需要多少存储空间。

已验证的基线来自 `linux-orangepi-h6-A53`、GCC `11.3.rel1`、`armv8` 的一次成功上板运行。该次运行的进程墙钟时间为 `0.542 s`，退出码为 `0`，但它只有一次样本，包含进程启动、输入准备、输出和两个 case，适合作为 smoke 通过记录，不能作为统计意义上的性能结论。

现有 Linux benchmark 用 `steady_clock` 取微秒时间，并在每个 case 的重复循环外计时。`RESULT` 的第三列因此是**该 case 全部重复次数的总耗时（µs）**，不是算法计算出的体积，也不是单次调用耗时。

| case | 重复次数 | 当前 `RESULT` 总耗时 | 估算单次耗时 | 计时范围 |
| --- | ---: | ---: | ---: | --- |
| `aabb_prism_volume` | 100 | 1,227 µs | 12.27 µs/op | 100 次 AABB 计算 |
| `convex_hull_prism_volume` | 20 | 35,749 µs | 1,787.45 µs/op | 20 次凸包体积计算 |

这两个 case 使用同一个确定性的闭合棱柱点云：`20 × 20 × 2 = 800` 点。点云构造发生在计时前，函数内部会校验计算体积是否符合参考值；目前 stdout 不输出体积数值。因此 `1227` 和 `35749` 是可比较的耗时，不是两个不同的体积结果。

## 建议分层采集

不要用一个数字同时代表所有性能。至少区分以下三层，并在报告中标明层级：

| 层级 | 包含内容 | 适用问题 |
| --- | --- | --- |
| `kernel_hot` | 已构造的点云上的单个算法调用，不含日志与初始化 | 算法本体是否退化 |
| `pipeline_warm` | 输入解析、滤波/下采样、分割、体积计算和结果整理 | 真实业务帧的处理性能 |
| `process_cold` | ELF 启动、动态装载、初始化、完整 pipeline 和退出 | 板端一次命令的用户可感知时延 |

当前 `RESULT` 接近 `kernel_hot`，而 `0.542 s` 更接近一次 `process_cold` 观测。两者都应保留，但不应混合成同一条回归阈值。

## 延迟与吞吐指标

每个 case 建议先执行 10 次以上 warm-up，再收集至少 30 个独立样本；若要以 p99 作为门禁，样本数应提高到 100 以上。每次采集应保存原始样本，而不是只保存平均值。

每个样本至少输出：

| 指标 | 单位 | 说明 |
| --- | --- | --- |
| `latency_us` | µs | 单次调用或单帧完整 pipeline 的墙钟耗时 |
| `min_us` / `mean_us` / `median_us` | µs | 最小值、均值和中位数 |
| `p95_us` / `p99_us` | µs | 尾延迟；同时记录样本量 |
| `stddev_us` | µs | 抖动程度，不以它替代分位数 |
| `throughput_points_s` | points/s | `输入点数 / latency_s` |
| `throughput_frames_s` | frames/s | 完整 pipeline 的帧率 |
| `repeat_count` | 次 | 原始计时覆盖的循环次数，避免把总耗时误当单次耗时 |

建议改为“单次采样 + 多样本”的结构：保留现有 case 内循环用于降低计时器量化误差，同时在外层重复独立采样。对于点云算法，还应分别报告输入点数、下采样后的点数、ROI/托盘尺寸、配置文件 SHA-256 和随机种子；否则不同版本的吞吐数字没有可比性。

建议优先使用 `clock_gettime(CLOCK_MONOTONIC_RAW)`；若目标系统不支持则使用 `CLOCK_MONOTONIC`，并在元数据中记录实际时钟源。计时区域中不要写 stdout、分配测试数据或做文件 I/O，除非该指标明确属于 `pipeline_warm` 或 `process_cold`。

## RAM 指标

Linux 上的“RAM”不应只报一次 `VmRSS`。推荐按每个 case 使用独立子进程，避免前一个 case 的分配、页缓存或 allocator 状态污染后一个 case，然后记录：

| 指标 | 含义 | 推荐来源 |
| --- | --- | --- |
| `rss_kib` | 采样时常驻集 | `/proc/<pid>/status` 的 `VmRSS` |
| `peak_rss_kib` | 该子进程生命周期峰值常驻集 | `getrusage(RUSAGE_SELF).ru_maxrss` 或 `VmHWM` |
| `pss_kib` | 按共享页比例分摊后的内存 | `/proc/<pid>/smaps_rollup` 的 `Pss` |
| `uss_kib` | 私有内存近似值 | `Private_Clean + Private_Dirty`，注明为近似值 |
| `heap_in_use_bytes` | 堆分配器视角的已用堆 | 可选 allocator 统计；不能替代 RSS |
| `ram_delta_kib` | 相对空闲基线的增量 | 进程启动后、case 前后分别采样 |

`/usr/bin/time -v ./benchmark` 可快速给出整进程最大 RSS，但只能得到整个程序的聚合结果。对于短于采样周期的 case，轮询 `/proc` 往往会漏掉峰值；应优先使用 `ru_maxrss` 或由受控 cgroup v2 读取 `memory.peak`。报告中应同时给出空闲基线、均值 RSS、峰值 RSS/HWM 和样本量。

## ROM、ELF 与部署体积

Linux A 核的“ROM”应明确为设备文件系统的部署占用，而不是只报一个 ELF 文件大小。每次构建建议收集：

| 指标 | 命令或来源 | 解释 |
| --- | --- | --- |
| `elf_file_bytes` | `stat -c%s benchmark` | 可执行文件落盘大小 |
| `text/data/bss` | `size -A benchmark` | 代码、已初始化数据和零初始化数据的段规模 |
| `stripped_elf_bytes` | strip 后的副本 | 发布可执行文件的实际大小，需保留带符号工件供调试 |
| `dynamic_deps` | `readelf -d`、`ldd` 或动态加载器 `--list` | 动态依赖集合与解析结果 |
| `dependency_closure_bytes` | ELF 加其实际部署的动态库 | 目标根文件系统新增占用 |
| `bundle_bytes` / `bundle_compressed_bytes` | 传输包统计 | 板测传输和归档成本 |

`bss` 在 ELF 文件中通常不占等量的 flash/文件系统空间，但会在装载后占用 RAM，因此必须单列，不能把它直接加进 ROM。还应记录静态/动态链接方式、是否 strip、链接器和动态解释器，以免两个大小相近但部署依赖完全不同的工件被误判为等价。

## CPU、系统与热状态

板端性能波动通常首先来自系统状态，而非算法改动。每组样本应附带：

- CPU 型号、核心数、内核版本、OS 镜像标识与启动参数。
- CPU affinity（例如 `taskset`）、实际运行核心、governor 和采样期间的频率范围。
- 启动前后温度、热限频/降频状态、可用内存和主要后台负载。
- user CPU time、system CPU time、上下文切换、CPU migration、page fault。
- 在系统权限允许时的 `perf stat`：`task-clock`、`cycles`、`instructions`、`cache-misses`、`context-switches`、`cpu-migrations`、`page-faults`。若内核不允许 perf，报告为“不支持”，不要伪造零值。

采集回归基线时，建议固定 CPU affinity，设定已记录的 governor，等待温度稳定，并避免在同一板上并行跑其他 board job。性能报告需要记录这些控制条件，才能区分真实回归和频率/温度抖动。

## 正确性、输入与可追溯性

性能数字只有在功能结果正确时才有效。每条样本或样本组至少关联以下元数据：

```json
{
  "git_sha": "<source commit>",
  "conan_ref": "food3d_im/0.1.0#<recipe-revision>:<package-id>#<package-revision>",
  "target_key": "linux-orangepi-h6-A53",
  "toolchain": "arm-toolchain/11.3.rel1 (GCC 11)",
  "fixture_id": "<name>",
  "fixture_sha256": "<sha256>",
  "input_points": 800,
  "config_sha256": "<sha256>",
  "random_seed": 0,
  "case": "convex_hull_prism_volume"
}
```

除计时外，建议为每个 fixture 单独保存预期体积、实际体积、绝对/相对误差、容差和成功率。生产数据至少覆盖规则棱柱、凹形食物、噪声/离群点、稀疏点云、大点云和空/退化输入。这样才能发现“性能变好但精度下降”的回归。

## 输出格式与兼容策略

现有协议是 watcher 的稳定合同，应继续保留：

```text
BENCHMARK_START
MODULE|food3d_im|cases=2
RESULT|<case>|<raw_total_us>
BENCHMARK_END
```

不要改变 `RESULT` 行的字段顺序或单位含义。建议在同一次运行旁路生成 `benchmark-metrics.jsonl`（每行一条样本）和 `benchmark-summary.json`（分位数汇总），而不是把复杂字段塞进旧协议。JSONL 中可包含 `latency_us`、`rss_kib`、`peak_rss_kib`、`input_points`、频率、温度和全部溯源字段；watcher 保持解析旧协议，后续再按 schema 读取新工件。

## 初始质量门禁

在收集至少 30 组受控基线前，不建议设绝对毫秒阈值。先使用相对门禁，并把基线版本固定在已验证的 Conan package revision：

| 类别 | 初始建议 | 前提 |
| --- | --- | --- |
| 正确性 | 预期体积通过、误差不超过 fixture 容差 | 每次必测 |
| 中位延迟 | 相对基线退化不超过 5% | 同 fixture、同系统状态、样本数 ≥ 30 |
| p95 延迟 | 相对基线退化不超过 10% | 同上 |
| 峰值 RSS/HWM | 相对基线增长不超过 10% | 每 case 独立进程或 cgroup 统计 |
| ELF/部署闭包 | 增长不超过 5%，且无未审计新动态依赖 | 明确 strip/link 策略 |
| 稳定性 | 无超时、无热限频告警、无异常 page fault 峰值 | 每次板测 |

阈值应按算法、fixture 和目标板分别维护；A53 的基线不能直接用于 A55/A76 或桌面构建。

## 推荐落地顺序

1. 在现有输出的 metadata 中补充 `unit=us`、`repeat_count`、输入点数和 fixture hash，并把当前 `RESULT` 命名为 `raw_total_us`。
2. 为每个 case 增加 warm-up、30 次独立采样和 JSONL 原始样本输出；保留现有文本标记以兼容 watcher。
3. 用独立子进程或 cgroup 采集 `ru_maxrss`/`memory.peak`，同时采集进程级 user/system CPU time。
4. 在构建产物中增加 `size -A`、动态依赖闭包和 bundle 大小清单。
5. 连续积累受控板端基线后，再启用延迟、RSS 和部署大小的相对回归门禁。

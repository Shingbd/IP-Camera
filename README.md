# 智能安防哨兵监控系统

基于 RK3566 + GStreamer + RKNN 的实时 AI 安防监控系统。支持 YOLOv8-pose 人体关键点检测、行为分析（蹲伏/暴力/聚集）、RTSP 推流、异常自动录制。

## 硬件

- SoC: RK3566 (Cortex-A55 ×4, 1 × NPU)
- 摄像头: OV5695 (2592×1944, 通过 `/dev/video*` 接入)
- 本地显示: HDMI / 显示屏 (通过 autovideosink)

## 依赖

| 依赖 | 说明 |
|------|------|
| GStreamer ≥1.16 | `gstreamer1.0`, `gstreamer-app-1.0`, `gstreamer-rtsp-server-1.0` |
| RKNN Toolkit | `librknnrt.so`, `rknn_api.h` |
| RKNN Model Zoo | 位于 `3rdpart/rknn_model_zoo/` |
| GCC ≥8 | C++17 |

## 构建

```bash
git clone <repo>
cd Main_project
mkdir build && cd build
cmake ..
make -j4
```

## 使用

```bash
./cam                          # 默认 /dev/video10
./cam -d 0                     # 使用 /dev/video0
./cam model/yolov8_pose.rknn   # 指定模型路径
./cam -d 0 model/custom.rknn   # 同时指定设备和模型
```

### RTSP 推流

启动后输出 RTSP 地址，可用 VLC / ffplay 拉流：

```
RTSP stream ready at rtsp://192.168.x.x:7551/test
```

```bash
ffplay rtsp://192.168.x.x:7551/test
```

## 架构

两条独立 GStreamer 管道（解决 autovideosink + appsink 在同一管道中的 preroll 死锁问题）：

```
┌─ 摄像头管道 ─────────────────────────────────────┐
│ v4l2src → videoconvert → videoscale → NV12(640×480) │
│     → tee                                           │
│         ├─ appsink1 (AI + 本地显示线程)               │
│         └─ appsink2 (RTSP 推流线程)                   │
└─────────────────────────────────────────────────────┘

┌─ 显示管道 ───────────────────────┐
│ appsrc → videoconvert → autovideosink │
└──────────────────────────────────────┘

┌─ RTSP 管道 ──────────────────────────────────┐
│ appsrc → videoconvert(I420) → mpph264enc → rtph264pay │
└──────────────────────────────────────────────────────┘
```

### 关键设计

- **双管道**: 摄像头采集与本地显示分离，避免 autovideosink 阻塞 appsink 的数据流
- **零拷贝 RTSP**: tee 直接共享 `GstBuffer`，RTSP 线程只做 `gst_buffer_ref` 不拷贝
- **深拷贝 AI**: tee 共享的内存不可写，AI 线程必须 `gst_buffer_copy` 才能绘制
- **NV12**: 全链路使用 NV12（摄像头输出格式），x264enc/mpph264enc 原生支持

## 功能

### 实时人体检测

- YOLOv8-pose 模型推理，绘制 17 个关键点和 19 条骨架连线
- 显示置信度分数

### 行为分析 (behavior_detection)

| 行为 | 算法 | 阈值 |
|------|------|------|
| 蹲伏 Crouch | 关键点比例 (髋-踝)/(肩-踝) < 0.45 | `crouching_conf > 0.5` |
| 暴力 Violence | 框 IOU > 0.3 或 手腕高于肩膀 | `violence_conf > 0.4` |
| 聚集 Crowd | 人数 ≥ 阈值 | 默认 4 人 |

### 异常录制

三段式状态机：

```
IDLE → WARMING (检测到异常)
         │ 持续 ≥3s → ACTIVE (开始录制, xxx秒去抖前取消)
         │ 连续~1s正常 → 回到 IDLE
ACTIVE
         │ 连续~2s正常 + 已录≥5s → 停止录制, 回到 IDLE
         │ 录制不足5s → 安全门禁, 不停止
```

- 格式: H.264 + MP4 (`record_YYYYMMDD_HHMMSS.mp4`)
- 文件内包含 AI 叠加画面（关键点+行为文字+REC标记）
- 编码器: `mpph264enc`（RK3566 硬件编码）

### 画面叠加

帧上实时显示（从左上开始排列）：

```
people=3                    (绿色)
CROWD ALERT                 (红色, >4人)
CROUCH                      (黄色)
VIOLENCE                    (红色)
REC                         (红色, 右上角, 录制中)
```

## 命令行参数

| 参数 | 简写 | 默认值 | 说明 |
|------|------|--------|------|
| `--port` | `-p` | `7551` | RTSP 监听端口 |
| `--device` | `-d` | `10` | 摄像头设备号 (`/dev/videoNUM`) |
| `(位置参数)` | | `model/yolov8_pose.rknn` | RKNN 模型路径 |

## 模型

默认模型: `model/yolov8_pose.rknn`（YOLOv8-pose，人体关键点检测）

## 文件结构

```
Main_project/
├── cam.cpp                 # 主程序 (管道创建/AI推理/录制/RTSP)
├── behavior_detection.h    # 行为分析接口
├── behavior_detection.cpp  # 行为分析实现 (蹲伏/暴力/聚集)
├── CMakeLists.txt          # 构建配置
├── 3rdpart/
│   └── rknn_model_zoo/     # RKNN 模型库 (postprocess/image_utils/image_drawing)
├── docs/
│   ├── DEBUG_SUMMARY.md    # 调试日志
│   └── RKNN_FLOW.md        # RKNN 推理流程文档
└── README.md
```

## 注意事项

- OV5695 输出 2592×1944，`videoscale` 负责缩放到 640×480
- RK3566 只有 1 个 NPU 核心，不需要 `rknn_set_core_mask`
- 系统需要提前加载 `rknpu` 内核驱动
- 确保 `/dev/video*` 有读写权限（可 `sudo chmod 666 /dev/video*`）

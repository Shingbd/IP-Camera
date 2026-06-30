# AGENTS.md — 智能安防哨兵监控系统 (终端 A)

## Build & Run

```bash
mkdir build && cd build && cmake .. && make -j4
./cam                                    # /dev/video10, model/yolov8_pose.rknn
./cam -d 0 -p 8551 model/custom.rknn    # device, port, model
```

No test suite, no linter, no typecheck.

## Project Structure

`src/` — 源文件 | 角色
---|---
`cam.cpp` | Entrypoint — pipelines, AI loop, RTSP server, recording state machine
`behavior_detection.h/.cpp` | Crouch / violence / crowd analysis
`alert_server.h/.cpp` | TCP AlertServer — 加密收发告警命令
`crypto_utils.h/.cpp` | AES-256-GCM 加解密

`3rdpart/rknn_model_zoo/` — RKNN libs (git submodule)

## RTSP

- Default port 7551, mount `/test` → `rtsp://<ip>:7551/test`
- RTSP thread pulls from `sink_rtsp`, pushes `gst_buffer_ref` to each connected client.

## Key Constants

- `FRAME_W=640, FRAME_H=480`
- `crowd_threshold=4`, `crouching_conf>0.5`, `violence_conf>0.4`.
- AlertServer port: `-a <port>` (default 7552)

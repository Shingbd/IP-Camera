# AGENTS.md — 智能安防哨兵监控系统

## Build & Run

```bash
mkdir build && cd build && cmake .. && make -j4
./cam                                    # /dev/video10, model/yolov8_pose.rknn
./cam -d 0 -p 8551 model/custom.rknn    # device, port, model
```

No test suite, no linter, no typecheck.

## Project Structure

| File | Role |
|------|------|
| `cam.cpp` | Entrypoint — pipes, AI loop, RTSP server, recording state machine |
| `behavior_detection.h/cpp` | Crouch / violence / crowd analysis |
| `3rdpart/rknn_model_zoo/` | RKNN libs (postprocess, image_utils, image_drawing) — git submodule |
| `CMakeLists.txt` | C++17, links GStreamer + rknn_model_zoo utils |

## Architecture

**Two separate GStreamer pipelines** (single pipeline with autovideosink + appsink causes preroll deadlock):

- **Camera pipe**: `v4l2src → videoconvert → videoscale → NV12 640×480 → tee → appsink_ai + appsink_rtsp`
- **Display pipe**: `appsrc → videoconvert → autovideosink`
- **RTSP clients** (dynamic): `appsrc → videoconvert(I420) → mpph264enc → rtph264pay`

## Critical Gotchas

- **tee shares memory, not writable** — AI thread must `gst_buffer_new_and_alloc + memcpy` (deep copy) before drawing. RTSP thread uses zero-copy `gst_buffer_ref`.
- **NV12 throughout** (not BGR/RGB). Stride = width. Use `IMAGE_FORMAT_YUV420SP_NV12` in `image_buffer_t`.
- **Caps can be NULL** from `gst_sample_get_caps` — always fallback to `gst_pad_get_current_caps`. Never dereference a NULL caps.
- **`gst_structure_get_string` returns internal pointer** — copy to `std::string` before `gst_caps_unref`.
- **OV5695 outputs 2592×1944** — `videoscale` in camera pipe shrinks to 640×480.
- **RK3566 has 1 NPU core** — do NOT call `rknn_set_core_mask`.
- **Model is INT8 quantized** — input must be uint8 with `pass_through=0`. Model expects RGB888 640×640 letterbox-padded image.
- **Recording state machine**: `IDLE → WARMING(3s abnormal) → ACTIVE → normal≥2s + recorded≥5s → IDLE`. Safety hold prevents <5s clips.
- **`get_local_ip()` iterates `getifaddrs`** — skips `lo`, returns first non-loopback AF_INET address.

## RTSP

- Default port 7551, mount `/test` → `rtsp://<ip>:7551/test`
- RTSP thread pulls from `sink_rtsp`, pushes `gst_buffer_ref` to each connected client.

## Key Constants

- `FRAME_W=640, FRAME_H=480` — single source of truth, used everywhere.
- `crowd_threshold=4`, `crouching_conf>0.5`, `violence_conf>0.4`.
- YOLOv8-pose: 17 COCO keypoints. Skeleton array `g_skeleton[38]` defined in `cam.cpp:56`.

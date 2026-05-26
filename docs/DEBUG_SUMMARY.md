# 调试问题全记录

## 1. 管道状态死锁 — PAUSED 卡住不进 PLAYING

### 现象
- 日志只输出 `BUS state: NULL → READY`、`BUS state: READY → PAUSED`
- 永远看不到 `PAUSED → PLAYING`
- 3 秒看门狗持续提示 "waiting for camera frames..."
- RTSP 服务器已启动，但 RTSP 和 AI 线程都拉不到帧

### 原因
GStreamer 管道状态机的规则：**所有 sink 元素（数据末端）都必须拿到第一帧（preroll buffer）才算准备好，管道才会从 PAUSED 进入 PLAYING。**

老版本是一个管道包含三个 sink：
```
v4l2src → tee ─→ queue → appsink_ai       ← 需要 AI 线程 pull_sample() 才能 preroll
               └→ queue → appsink_rtsp     ← 需要 RTSP 线程 pull_sample() 才能 preroll
appsrc → capsfilter → autovideosink        ← 需要 appsrc 收到推帧才能 preroll
```

形成循环依赖：
```
autovideosink 等 appsrc 推帧 → appsrc 等 AI 线程 pull 完再推
→ AI 线程等管道到 PLAYING 才开始 pull → 管道被 autovideosink 卡在 PAUSED
```

### 修复
**拆成两个独立管道**。每个管道各自独立走状态机，互不阻塞。

```cpp
// ── 管道 1: 摄像头采集（只有 appsink，没有 autovideosink）──
const std::string cam_pipe =
    "v4l2src device=" + camera_path + " ! "
    "videoconvert ! videoscale ! "
    "video/x-raw,format=NV12,width=" + W + ",height=" + H + " ! "
    "tee name=t "
    "t. ! queue ! videoconvert ! video/x-raw,format=NV12 ! "
    "    appsink name=sink_ai max-buffers=1 drop=true "
    "t. ! queue ! videoconvert ! video/x-raw,format=NV12 ! "
    "    appsink name=sink_rtsp max-buffers=1 drop=true";

// ── 管道 2: 本地显示（只有 appsrc + autovideosink）──
const std::string display_pipe =
    "appsrc name=local_src is-live=true format=time ! "
    "video/x-raw,format=NV12,width=" + W + ",height=" + H + " ! "
    "videoconvert ! "
    "autovideosink sync=false";

// 分别创建和启动
GstElement *cam_pipeline = gst_parse_launch(cam_pipe.c_str(), &err);
GstElement *display_pipeline = gst_parse_launch(display_pipe.c_str(), &err);

gst_element_set_state(cam_pipeline, GST_STATE_PLAYING);
gst_element_set_state(display_pipeline, GST_STATE_PLAYING);
```

摄像头管道只有两个 appsink→线程 pull_sample 立刻 preroll→管道顺畅进入 PLAYING→v4l2src 开始推流。显示管道独立运行不受影响。

---

## 2. tee 共享内存不可写

### 现象
```
(cam:3150): GStreamer-CRITICAL **: 21:06:35.081: write map requested on non-writable buffer
```

### 原因
`tee` 的工作方式：**不拷贝数据**。它把同一块 `GstMemory` 分发给所有下游分支。每个分支拿到不同的 `GstBuffer`，但底层 `GstMemory` 被多个 GstBuffer 共享（refcount > 1）。

```
v4l2src → tee ─→ appsink_ai   → GstBuffer#1 → GstMemory#A (refcount=2)
               └→ appsink_rtsp → GstBuffer#2 → GstMemory#A (refcount=2)
```

当 AI 线程试图 `gst_buffer_map(buf, &info, GST_MAP_WRITE)` 时，GStreamer 发现底层内存 refcount > 1，**拒绝给写权限**——防止写入污染另一个分支正在读的数据。

### 修复
AI 线程里 deep-copy 一份独立的 buffer，在自己的副本上画框。RTSP 路径不修改数据，仍然是 `gst_buffer_ref` 零拷贝推送。

```cpp
// 获取原始 buffer 并 deep-copy
GstBuffer *orig = gst_sample_get_buffer(sample);

GstMapInfo src_info;
gst_buffer_map(orig, &src_info, GST_MAP_READ);
GstBuffer *buf = gst_buffer_new_and_alloc(src_info.size);   // 新分配内存
GstMapInfo dst_info;
gst_buffer_map(buf, &dst_info, GST_MAP_WRITE);
memcpy(dst_info.data, src_info.data, src_info.size);         // 拷数据
gst_buffer_unmap(buf, &dst_info);
gst_buffer_unmap(orig, &src_info);

// 复制时间戳
GST_BUFFER_PTS(buf) = GST_BUFFER_PTS(orig);
GST_BUFFER_DTS(buf) = GST_BUFFER_DTS(orig);
GST_BUFFER_DURATION(buf) = GST_BUFFER_DURATION(orig);

gst_sample_unref(sample);   // 释放原始 buffer（RTSP 那边有自己的 ref，不受影响）

// 现在 buf 是独立副本，可以随意写
gst_buffer_map(buf, &info, GST_MAP_READ | GST_MAP_WRITE);   // 可以写了！
// ... 画框/画骨架 ...
gst_buffer_unmap(buf, &info);
gst_app_src_push_buffer(GST_APP_SRC(local_src), buf);        // 推给本地显示
```

RTSP 线程不改：
```cpp
// 只 ref 不 copy，零拷贝
gst_app_src_push_buffer(GST_APP_SRC(it->appsrc), gst_buffer_ref(buf));
```

---

## 3. caps 为 NULL 导致连锁崩溃

### 现象
```
(cam:3150): GStreamer-CRITICAL **: ...: gst_caps_get_structure: assertion 'GST_IS_CAPS (caps)' failed
(cam:3150): GStreamer-CRITICAL **: ...: gst_structure_get_int: assertion 'structure != NULL' failed
(cam:3150): GStreamer-CRITICAL **: ...: gst_mini_object_unref: assertion 'GST_MINI_OBJECT_REFCOUNT_VALUE ... > 0' failed
```

### 原因
`gst_sample_get_caps(sample)` 在特定条件下可能返回 NULL（比如第一帧 sample 没有附带的 caps，或管道协商尚未完成）。老代码直接解引用：

```cpp
// 老代码 — 三个问题
GstCaps *caps = gst_sample_get_caps(sample);           // ← 可能 NULL
GstStructure *s = gst_caps_get_structure(caps, 0);     // ← NULL 传入，崩
gst_caps_unref(caps);                                   // ← NULL 传入，再崩
```

### 修复
1. 检查 NULL，fallback 到 appsink pad 上拿协商好的 caps
2. 所有操作包在 `if (caps)` 里
3. 宽高有默认 fallback 值

```cpp
GstCaps *caps = gst_sample_get_caps(sample);
if (!caps) {
    // fallback：从 appsink 的 sink pad 拿实际协商好的 caps
    GstPad *pad = gst_element_get_static_pad(GST_ELEMENT(sink_ai), "sink");
    if (pad) {
        caps = gst_pad_get_current_caps(pad);
        gst_object_unref(pad);
    }
}

int w = FRAME_W, h = FRAME_H;   // 有默认值保底
std::string fmt_str = "?";
if (caps) {
    GstStructure *s = gst_caps_get_structure(caps, 0);
    if (s) {
        gst_structure_get_int(s, "width", &w);
        gst_structure_get_int(s, "height", &h);
        const char *f = gst_structure_get_string(s, "format");
        if (f) fmt_str = f;
    }
    gst_caps_unref(caps);   // 只在非 NULL 时 unref
}
```

---

## 4. 野指针（dangling pointer）

### 现象
偶然性的崩溃，或打印出乱码的摄像头格式名。

### 原因
```cpp
const char *fmt = gst_structure_get_string(s, "format");  // ← 指向 caps 内部
gst_caps_unref(caps);                                       // ← caps 释放，fmt 变野指针
std::cout << fmt << std::endl;                              // ← 读已释放的内存
```

### 修复
用 `std::string` 拷贝字符串值：
```cpp
const char *f = gst_structure_get_string(s, "format");
if (f) fmt_str = f;   // std::string fmt_str，拷贝值，不持有指针
```
`gst_caps_unref(caps)` 之后 `fmt_str` 仍然有效。

---

## 5. BGR → RGB 通道错乱 + Stride 不对 → 画面条纹

### 现象
autovideosink 显示的窗口里有画面，但全是彩色条纹，无法辨认。

### 原因
两个问题叠加：

**问题 A — 像素格式错乱**
```cpp
src_img.format = IMAGE_FORMAT_RGB888;  // 告诉 image_drawing.c 这是 RGB
```
但 GStreamer 给的是 `video/x-raw,format=BGR`，字节顺序是 `[B][G][R]`。image_drawing.c 按 `[R][G][B]` 处理，红蓝通道互换。

**问题 B — Stride 算错**
```cpp
src_img.width_stride = w * 3;   // = 1920
```
如果 videoconvert 实际输出的 stride 带对齐填充（比如 2048），绘制函数计算像素位置时：
```
第 Y 行的像素地址 = virt_addr + Y × width_stride = virt_addr + Y × 1920  ← 错的！
实际位置         = virt_addr + Y × 2048
```
每行偏移少了 128 字节，在错误位置写数据→覆盖原始画面→斜向条纹。

### 修复
全线从 BGR 改成 NV12。

**NV12 的好处：**
- 每像素 1.5 字节比 BGR 的 3 字节节省一半带宽
- autovideosink/kmssink 原生支持 NV12
- x264 编码器原生支持 NV12
- RK3566 的 RGA 硬件加速支持 NV12
- NV12 的 stride = 像素宽度（`w`），不需要 `w * 3`

```cpp
// 摄像头管道 caps：BGR → NV12
"video/x-raw,format=NV12 ! appsink ..."

// 显示管道 caps：
"appsrc name=local_src is-live=true format=time ! "
"video/x-raw,format=NV12,width=640,height=480 ! ..."

// AI 线程 image_buffer_t 设置：
image_buffer_t src_img;
src_img.width = w;
src_img.height = h;
src_img.width_stride = w;                    // NV12：stride = 像素宽度
src_img.height_stride = h;
src_img.format = IMAGE_FORMAT_YUV420SP_NV12;  // 不再是 RGB888
src_img.virt_addr = info.data;
```

全部涉及 BGR 的地方同步改成 NV12 的完整清单：
- 摄像头管道 tee 分支的 `video/x-raw,format=BGR` → `NV12`
- 显示管道的 `format=BGR` → `format=NV12`
- RTSP 工厂启动串的 `format=BGR` → `format=NV12`
- `media_configure_cb` 和 `main` 中设置 appsrc caps 的 `"BGR"` → `"NV12"`
- `image_buffer_t.format` → `IMAGE_FORMAT_YUV420SP_NV12`
- `width_stride` → `w`（不再是 `w * 3`）

---

## 6. 摄像头分辨率不匹配

### 现象
```
▲ first frame: 2592x1944 NV12
```
摄像头输出 2592×1944，但代码硬编码 640×480。推 2592×1944 的 buffer 到 caps 为 640×480 的 appsrc → caps 不匹配 → 画面绿条纹。

### 原因
OV5695 摄像头默认输出 2592×1944 分辨率。

### 修复
摄像头管道加 `videoscale` 缩到 640×480：

```cpp
const std::string cam_pipe =
    "v4l2src device=" + camera_path + " ! "
    "videoconvert ! videoscale ! "                                  // ← 新增 videoscale
    "video/x-raw,format=NV12,width=" + W + ",height=" + H + " ! "  // ← 这里强制 640x480
    "tee name=t "
    "t. ! queue ! videoconvert ! video/x-raw,format=NV12 ! "
    "    appsink name=sink_ai max-buffers=1 drop=true "
    "t. ! queue ! videoconvert ! video/x-raw,format=NV12 ! "
    "    appsink name=sink_rtsp max-buffers=1 drop=true";
```

数据流：
```
v4l2src (2592×1944 YUYV) → videoconvert → videoscale → 640×480 NV12 → tee → 两路 appsink
```

所有下游统一 640×480，caps 完全匹配，不再绿屏。

---

## 7. 分辨率统一管理

### 现象
硬编码 640/480 散落代码各处，改分辨率要改 5 个地方。

### 修复
```cpp
constexpr int FRAME_W = 640;
constexpr int FRAME_H = 480;
```
所有用到分辨率的地方（显示管道 caps、RTSP 管道 caps、AI fallback、RTSP 客户端 caps）全部引用这两个常量。改分辨率只需改一行。

---

## 8. 可配摄像头设备号

### 修改
```cpp
static int device_num = 10;

static GOptionEntry entries[] = {
    {"port", 'p', 0, G_OPTION_ARG_STRING, &port,
        "Port to listen on (default: " DEFAULT_RTSP_PORT ")", "PORT"},
    {"device", 'd', 0, G_OPTION_ARG_INT, &device_num,
        "Camera device number (default: 10 → /dev/video10)", "NUM"},
    {NULL}
};
```

使用：
```bash
./cam -d 0              # 打开 /dev/video0
./cam -d 10             # 打开 /dev/video10（默认）
./cam -d 0 -p 8551     # 同时改设备号和 RTSP 端口
```

---

## 最终架构图

```
┌── 管道 1（摄像头）──────────────────────────────────────────────────┐
│ v4l2src (/dev/video0)                                   2592×1944 │
│   ↓ videoconvert                                                   │
│   ↓ videoscale                                                     │
│   ↓ video/x-raw,format=NV12,width=640,height=480          640×480 │
│   ↓ tee                                                             │
│   ├── queue → appsink_ai    ← AI 线程 pull → deep-copy → 画框      │
│   └── queue → appsink_rtsp  ← RTSP 线程 pull → ref → RTSP 客户端   │
└────────────────────────────────────────────────────────────────────┘

┌── 管道 2（本地显示）────────────────────────────────┐
│ appsrc (NV12 640×480) → autovideosink                │
│   ↑ AI 线程 push_buffer(画框后)                        │
└──────────────────────────────────────────────────────┘

┌── RTSP 客户端管道 × N（由 gst_rtsp_server 动态创建）──┐
│ appsrc (NV12 640×480) → x264enc → rtph264pay         │
│   ↑ RTSP 线程 push_buffer(原始画面 ref)               │
└──────────────────────────────────────────────────────┘
```

## 性能数据（RK3566, OV5695 @ 2592×1944 → 640×480）

```
▲ first frame: 640x480 NV12
rknn_run time=62ms, FPS ≈ 16        ← NPU 推理
post_process time=0.24ms, FPS ≈ 4200  ← 后处理几乎不占时间
```

NPU 推理每帧约 60ms，整体帧率受推理速度限制在 ~15fps。RTSP 推流也在同样帧率。

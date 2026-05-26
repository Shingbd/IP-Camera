# RKNN 模型加载与推理全流程（Rockchip NPU）

本文档覆盖从 `.rknn` 模型文件到推理结果的全过程，结合项目中的实际代码。

---

## 1. RKNN API 概述

RKNN 是 Rockchip NPU 的推理运行时 API，提供 5 个核心函数：

| 函数 | 作用 |
|---|---|
| `rknn_init` | 加载 `.rknn` 模型文件，创建推理上下文 |
| `rknn_query` | 查询模型信息（输入/输出数量、维度、格式、量化参数） |
| `rknn_inputs_set` | 设定推理输入数据 |
| `rknn_run` | 执行 NPU 推理 |
| `rknn_outputs_get` | 获取推理结果 |
| `rknn_outputs_release` | 释放输出缓冲区 |
| `rknn_destroy` | 销毁上下文，释放 NPU 资源 |

API 声明位于 `3rdpart/rknn_model_zoo/3rdparty/rknpu2/include/rknn_api.h`。

---

## 2. 完整流程

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│  1. 模型加载  rknn_init() → rknn_query()                                        │
│     读取 .rknn 文件，加载到 NPU，查询输入/输出 tensor 信息                        │
├──────────────────────────────────────────────────────────────────────────────────┤
│  2. 预处理    convert_image_with_letterbox()                                     │
│     摄像头帧 (NV12 640×480) → letterbox 缩放 + 补边 → RGB888 (640×640)           │
├──────────────────────────────────────────────────────────────────────────────────┤
│  3. 设置输入  rknn_inputs_set()                                                  │
│     把预处理后的数据传给 NPU                                                     │
├──────────────────────────────────────────────────────────────────────────────────┤
│  4. 推理      rknn_run()                                                         │
│     NPU 执行卷积计算，得到 4 个输出 tensor                                        │
├──────────────────────────────────────────────────────────────────────────────────┤
│  5. 获取输出  rknn_outputs_get()                                                 │
│     把 NPU 显存中的结果拷回 CPU 内存                                              │
├──────────────────────────────────────────────────────────────────────────────────┤
│  6. 后处理    post_process()                                                     │
│     解码输出 tensor → 边界框 + 关键点 → NMS → 坐标映射回原图                      │
├──────────────────────────────────────────────────────────────────────────────────┤
│  7. 画框      draw_rectangle() / draw_line() / draw_circle()                     │
│     在原图上绘制检测结果                                                          │
├──────────────────────────────────────────────────────────────────────────────────┤
│  8. 清理      rknn_outputs_release() → rknn_destroy()                            │
└──────────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. 各步骤详细代码

### 3.1 模型加载 — `init_yolov8_pose_model()`

**文件**: `3rdpart/rknn_model_zoo/examples/yolov8_pose/cpp/rknpu2/yolov8-pose.cc:43`

```cpp
int init_yolov8_pose_model(const char* model_path, rknn_app_context_t* app_ctx)
{
    // 1. 创建 RKNN 上下文（加载模型到 NPU）
    rknn_context ctx = 0;
    int ret = rknn_init(&ctx, (void*)model_path, 0, 0, NULL);
    // 参数: ctx, 模型路径或内存指针, size(0=从文件读), flag, extend

    // 2. 查询输入/输出数量（YOLOv8n-pose: 1 input, 4 outputs）
    rknn_input_output_num io_num;
    rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));

    // 3. 查询输入 tensor 属性
    rknn_tensor_attr input_attrs[io_num.n_input];
    for (int i = 0; i < io_num.n_input; i++) {
        input_attrs[i].index = i;
        rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &input_attrs[i], sizeof(rknn_tensor_attr));
        // 得到: dims=[1,640,640,3], fmt=NHWC, type=INT8, qnt_type=AFFINE
    }

    // 4. 查询输出 tensor 属性
    rknn_tensor_attr output_attrs[io_num.n_output];
    for (int i = 0; i < io_num.n_output; i++) {
        output_attrs[i].index = i;
        rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &output_attrs[i], sizeof(rknn_tensor_attr));
    }

    // 5. 保存到上下文
    app_ctx->rknn_ctx = ctx;
    app_ctx->io_num = io_num;
    app_ctx->input_attrs = input_attrs;   // 注意：实际代码中 malloc 复制
    app_ctx->output_attrs = output_attrs;
    app_ctx->model_width = 640;
    app_ctx->model_height = 640;
    app_ctx->model_channel = 3;
    app_ctx->is_quant = (input_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC);
}
```

**运行输出**：
```
model input num: 1, output num: 4
input tensors:
  index=0, name=images, n_dims=4, dims=[1, 640, 640, 3], n_elems=1228800,
  size=1228800, fmt=NHWC, type=INT8, qnt_type=AFFINE, zp=-128, scale=0.003922
output tensors:
  index=0, name=/model.22/Concat_1_output_0, dims=[1, 65, 80, 80]  ← 小尺度检测头
  index=1, name=/model.22/Concat_2_output_0, dims=[1, 65, 40, 40]  ← 中尺度检测头
  index=2, name=/model.22/Concat_3_output_0, dims=[1, 65, 20, 20]  ← 大尺度检测头
  index=3, name=/model.22/Concat_6_output_0, dims=[1, 17, 3, 8400] ← 关键点输出
```

---

### 3.2 预处理 — `convert_image_with_letterbox()`

**文件**: `3rdpart/rknn_model_zoo/utils/image_utils.c:699`

模型输入必须是 640×640 RGB888 正方形图。摄像头来的帧是 640×480 NV12（或其他分辨率），需要缩放 + 补黑边。

```
摄像头帧 640×480                 模型输入 640×640
┌──────────────┐                ┌──────────────────┐
│              │                │░░░░░░░░░░░░░░░░░░│  ← 黑边 (padding)
│   图像内容    │  ──scale──→   │  ┌────────────┐  │
│              │    +pad       │  │  图像内容    │  │
│              │                │  │            │  │
└──────────────┘                │  └────────────┘  │
                                │░░░░░░░░░░░░░░░░░░│  ← 黑边
                                └──────────────────┘
```

```cpp
int inference_yolov8_pose_model(rknn_app_context_t *app_ctx, image_buffer_t *img,
                                object_detect_result_list *od_results)
{
    letterbox_t letter_box;
    image_buffer_t dst_img;
    memset(&dst_img, 0, sizeof(dst_img));
    dst_img.width = app_ctx->model_width;     // 640
    dst_img.height = app_ctx->model_height;   // 640
    dst_img.format = IMAGE_FORMAT_RGB888;

    // 关键函数：把任意分辨率的输入转成 640×640 RGB888（letterbox 缩放+补边）
    int ret = convert_image_with_letterbox(img, &dst_img, &letter_box, 114);
    // 参数: 输入帧, 输出帧, letterbox参数(scale/offset), 背景色(114=灰色)

    // 之后 dst_img.virt_addr 指向 640×640×3 = 1228800 bytes 的 RGB 数据
}
```

`letter_box` 结构会记录缩放参数，后处理时需要用这些参数把坐标映射回原图：

```cpp
typedef struct {
    float scale;   // 缩放比例 (640/max(w,h))
    int x_pad;     // 水平补边像素数
    int y_pad;     // 垂直补边像素数
} letterbox_t;
```

例如 640×480 输入：`scale=640/640=1.0`, `x_pad=0`, `y_pad=80`（上下各补 80 行黑边）。

---

### 3.3 设置输入 — `rknn_inputs_set()`

```cpp
    // 预处理后的数据在 dst_img.virt_addr 中
    rknn_input inputs[1];
    inputs[0].index = 0;                    // 模型唯一输入 tensor
    inputs[0].buf = dst_img.virt_addr;      // 640×640×3 = 1228800 bytes
    inputs[0].size = dst_img.width * dst_img.height * dst_img.format;  // 640*640*3
    inputs[0].pass_through = 0;             // 0=让 NPU 做量化转换
    inputs[0].type = RKNN_TENSOR_UINT8;     // 8bit 无符号整型
    inputs[0].fmt = RKNN_TENSOR_NHWC;       // 通道在后 (HWC)

    rknn_inputs_set(app_ctx->rknn_ctx, 1, inputs);
```

**关键**：`pass_through = 0` 表示让 RKNN 驱动自动处理 INT8 量化。如果模型是 INT8 量化（`qnt_type == AFFINE_ASYMMETRIC`），驱动会根据 `zp=-128, scale=0.003922` 自动将 uint8 输入量化为 INT8 送给 NPU。如果 `pass_through = 1`，你需要自己量化数据。

---

### 3.4 推理 — `rknn_run()`

```cpp
    rknn_run(app_ctx->rknn_ctx, NULL);
    // 第二个参数 extend 通常传 NULL
    // NPU 执行异步推理，这个函数阻塞等待完成
```

输出日志：
```
rknn_run time=62.96ms, FPS = 15.88
```

RK3566 的 NPU 算力约 1 TOPS，YOLOv8n-pose 每帧推理约 60ms，理论帧率 ~16fps。

---

### 3.5 获取输出 — `rknn_outputs_get()`

```cpp
    rknn_output outputs[app_ctx->io_num.n_output];  // 4 个输出
    for (int i = 0; i < app_ctx->io_num.n_output; i++) {
        outputs[i].want_float = !app_ctx->is_quant;  // 量化模型 → 保留 INT8
    }

    rknn_outputs_get(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs, NULL);
```

YOLOv8n-pose 的 4 个输出：

| 索引 | 名称 | 形状 | 内容 |
|---|---|---|---|
| 0 | `/model.22/Concat_1_output_0` | `[1, 65, 80, 80]` INT8 | 小尺度检测（80×80 网格） |
| 1 | `/model.22/Concat_2_output_0` | `[1, 65, 40, 40]` INT8 | 中尺度检测（40×40 网格） |
| 2 | `/model.22/Concat_3_output_0` | `[1, 65, 20, 20]` INT8 | 大尺度检测（20×20 网格） |
| 3 | `/model.22/Concat_6_output_0` | `[1, 17, 3, 8400]` FP16 | 17×3 关键点 × 8400 个候选 |

**输出 0-2（检测头）**: 每个位置的前 64 维是 DFL（Distribution Focal Loss）box 参数，第 65 维是人脸置信度。`65 = 64 + 1`（1 个类别 person）。

**输出 3（关键点）**: 形状 `[17, 3, 8400]`。17 个关键点，每个关键点 3 个值 (x, y, confidence)。8400 = 80×80 + 40×40 + 20×20 是所有网格总和。

---

### 3.6 后处理 — `post_process()`

**文件**: `3rdpart/rknn_model_zoo/examples/yolov8_pose/cpp/postprocess.cc`

```cpp
int post_process(rknn_app_context_t *app_ctx, rknn_output *outputs,
                 letterbox_t *letter_box, float conf_threshold, float nms_threshold,
                 object_detect_result_list *od_results)
```

流程：

```
                 ┌──────────────────────────────┐
    output[0] → │ process_i8() 每个80×80网格    │
    output[1] → │ process_i8() 每个40×40网格    │  → 收集所有候选框 + 分数
    output[2] → │ process_i8() 每个20×20网格    │
                 └──────────────────────────────┘
                         ↓
                   按分数降序排序
                         ↓
                   NMS（非极大值抑制）
                   同类框 IOU > 0.4 则合并
                         ↓
                   ┌─────────────────────┐
    output[3] →    │ 读取关键点           │
                   │ 每个幸存框对应 8400   │
                   │ 个候选中的某一个      │
                   └─────────────────────┘
                         ↓
              ┌────────────────────────────┐
              │ 坐标映射回原图              │
              │ box.x = (x - x_pad) / scale │
              │ keypoint.x = ...            │
              └────────────────────────────┘
```

代码关键部分：

```cpp
// 每个检测网格处理
for (int i = 0; i < grid_size; i++) {  // 80×80, 40×40, 20×20
    int8_t *data = (int8_t *)output->buf;  // INT8 量化
    // 前 64 个值 → DFL 解码 → box (cx, cy, w, h)
    // 第 65 个值 → sigmoid → person 置信度
    if (score > conf_threshold) {
        // 保存到候选列表
    }
}

// NMS
std::sort(candidates, ...);  // 按分数降序
for (auto &c : candidates) {
    if (iou(c, existing) > nms_threshold) continue;
    keep.push_back(c);
}

// 关键点读取
int idx = keep[i].grid_index;  // 这个框对应哪个网格
float *kp_data = (float *)outputs[3].buf;  // FP16 关键点输出
for (int j = 0; j < 17; j++) {
    keypoints[j][0] = (kp_data[j * 3 + 0] - letter_box.x_pad) / letter_box.scale;
    keypoints[j][1] = (kp_data[j * 3 + 1] - letter_box.y_pad) / letter_box.scale;
    keypoints[j][2] = kp_data[j * 3 + 2];  // confidence
}
```

---

### 3.7 画框 — `image_drawing.c`

**文件**: `3rdpart/rknn_model_zoo/utils/image_drawing.c`

```cpp
// 画框
draw_rectangle(&src_img, x1, y1, w, h, COLOR_BLUE, 3);

// 画文字
sprintf(text, "person %.1f%%", det->prop * 100);
draw_text(&src_img, text, x1, y1 - 20, COLOR_RED, 10);

// 画骨架（19 条连线）
for (int j = 0; j < 19; j++) {
    int idx1 = g_skeleton[2*j] - 1;   // 起点关键点索引
    int idx2 = g_skeleton[2*j+1] - 1;  // 终点关键点索引
    draw_line(&src_img,
        (int)det->keypoints[idx1][0], (int)det->keypoints[idx1][1],
        (int)det->keypoints[idx2][0], (int)det->keypoints[idx2][1],
        COLOR_ORANGE, 3);
}

// 画关键点
for (int j = 0; j < 17; j++) {
    draw_circle(&src_img,
        (int)det->keypoints[j][0], (int)det->keypoints[j][1],
        3, COLOR_YELLOW, 2);
}
```

这些绘制函数支持 NV12（直接在 Y 和 UV 平面写入对应的亮度和色度值），不需要转成 RGB。

---

### 3.8 清理

```cpp
// 每帧推理后
rknn_outputs_release(app_ctx->rknn_ctx, app_ctx->io_num.n_output, outputs);

// 程序退出时
int release_yolov8_pose_model(rknn_app_context_t *app_ctx)
{
    free(app_ctx->input_attrs);
    free(app_ctx->output_attrs);
    rknn_destroy(app_ctx->rknn_ctx);
}
```

---

## 4. 本项目中的完整调用链

```
cam.cpp main()
  │
  ├─ init_yolov8_pose_model("model/yolov8_pose.rknn", &g_rknn_ctx)
  │    ├─ rknn_init()         ← NPU 加载模型
  │    ├─ rknn_query() × 5   ← 查 I/O tensor 属性
  │    └─ 存入 g_rknn_ctx
  │
  ├─ AI 线程（无限循环）
  │    │
  │    ├─ gst_app_sink_pull_sample(sink_ai)
  │    │   ↓
  │    ├─ deep-copy buffer（因 tee 共享内存不可写）
  │    │   ↓
  │    ├─ 构造 image_buffer_t {640,480,NV12,virt_addr}
  │    │   ↓
  │    ├─ inference_yolov8_pose_model(&g_rknn_ctx, &src_img, &results)
  │    │    ├─ convert_image_with_letterbox()  NV12→640×640 RGB888
  │    │    ├─ rknn_inputs_set()               设输入
  │    │    ├─ rknn_run()                      NPU 推理 (~60ms)
  │    │    ├─ rknn_outputs_get()              取 4 个输出
  │    │    ├─ post_process()                  解码 + NMS + 坐标映射
  │    │    └─ rknn_outputs_release()          释放输出
  │    │   ↓
  │    ├─ draw_rectangle / draw_line / draw_circle 画框画骨架
  │    │   ↓
  │    └─ gst_app_src_push_buffer(local_src)  推给本地显示
  │
  └─ release_yolov8_pose_model(&g_rknn_ctx)
       └─ rknn_destroy()   ← 释放 NPU 资源
```

---

## 5. 关键数据结构

### `rknn_app_context_t`（上下文）
```cpp
typedef struct {
    rknn_context rknn_ctx;               // NPU 上下文句柄
    rknn_input_output_num io_num;         // 输入(1) + 输出(4) 数量
    rknn_tensor_attr* input_attrs;        // 输入 tensor 属性数组
    rknn_tensor_attr* output_attrs;       // 输出 tensor 属性数组
    int model_channel;                    // 模型输入通道数 (3)
    int model_width;                      // 模型输入宽度 (640)
    int model_height;                     // 模型输入高度 (640)
    bool is_quant;                        // 是否是 INT8 量化模型
} rknn_app_context_t;
```

### `image_buffer_t`（图像缓冲区）
```cpp
typedef struct {
    int width;              // 像素宽度
    int height;             // 像素高度
    int width_stride;       // 行跨度（字节数）
    int height_stride;      // 行数
    image_format_t format;  // NV12 / RGB888 / ...
    unsigned char* virt_addr; // 数据指针
    int size;               // 总字节数
    int fd;                 // dma-buf fd（硬件加速用）
} image_buffer_t;
```

### `object_detect_result_list`（检测结果）
```cpp
typedef struct {
    int id;
    int count;
    object_detect_result results[128];  // 最多 128 个检测目标
} object_detect_result_list;

typedef struct {
    image_rect_t box;           // 边界框 {left, top, right, bottom}
    float keypoints[17][3];     // 17 个关键点 [x, y, confidence]
    float prop;                 // 置信度
    int cls_id;                 // 类别 ID (0=person)
} object_detect_result;
```

---

## 6. YOLOv8-pose 模型输出特点

YOLOv8-pose 的 4 个输出 tensor 中：

```
output[0-2]: 检测头 (INT8 量化)
  形状: [1, 65, grid_h, grid_w]
  65 = 64(DFL box) + 1(置信度)  ← 只有 1 个类别 (person)
  网格: 80×80, 40×40, 20×20 → 共 8400 个候选

output[3]: 关键点头 (FP16)
  形状: [1, 17, 3, 8400]
  17 个关键点:
    0=鼻, 1=左眼, 2=右眼, 3=左耳, 4=右耳,
    5=左肩, 6=右肩, 7=左肘, 8=右肘,
    9=左腕, 10=右腕, 11=左髋, 12=右髋,
    13=左膝, 14=右膝, 15=左踝, 16=右踝
  每个关键点: [x, y, confidence]
  8400 = 每个检测头网格总和
```

**后处理流程**：
1. 对 output[0-2] 的 8400 个候选分别解码 DFL → box + score
2. 阈值过滤（`BOX_THRESH = 0.5`）
3. NMS 合并重复框（`NMS_THRESH = 0.4`）
4. 对幸存框从 output[3] 中读取对应位置的关键点
5. 坐标通过 `letter_box.scale` 和 `letter_box.x_pad/y_pad` 映射回原图

---

## 7. 关键注意事项

### 7.1 量化模型输入格式
模型是 INT8 量化（`qnt_type = AFFINE_ASYMMETRIC, zp = -128, scale = 0.003922`），输入需要 **uint8**。`pass_through = 0` 时 RKNN 驱动自动做 uint8 → int8 量化转换：
```
int8_value = (uint8_value - 128) * scale  ← zp=-128 实际上是 zero-centered
```
因此传给 NPU 的 uint8 数据范围 [0, 255] 对应 int8 范围 [-128, 127]。

### 7.2 颜色格式
模型训练时用的是 RGB（如果是 COCO 预训练模型）。项目中 `convert_image_with_letterbox` 从 NV12 转成 RGB888（不是 BGR），与训练数据一致。

### 7.3 补边对坐标的影响
后处理输出的坐标在 640×640 模型空间（包含黑边）。映射回原图：
```
原图_x = (模型_x - x_pad) / scale
原图_y = (模型_y - y_pad) / scale
```

### 7.4 rknn_set_core_mask
RK3566 有 1 个 NPU 核心，不需要设置 core mask。如果是 RK3588（3 核 NPU），可以：
```cpp
rknn_set_core_mask(ctx, RKNN_NPU_CORE_0);  // 指定使用哪个核心
```

### 7.5 RNNT 运行时库
RK3566 需要 `librknnrt.so`（在 `3rdpart/rknn_model_zoo/3rdparty/rknpu2/lib/Linux/aarch64/` 下），运行时需要 LD_LIBRARY_PATH 包含该路径或把 .so 放到 `/usr/lib/`。

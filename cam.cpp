#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/video/video.h>
#include <iostream>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstring>
#include <string>
#include <thread>
#include <mutex>
#include <list>
#include <chrono>
#include <ctime>
#include <atomic>

#include "yolov8-pose.h"
#include "image_utils.h"
#include "image_drawing.h"
#include "behavior_detection.h"
#include "alert_server.h"

#define DEFAULT_RTSP_PORT "7551"

static char *port = (char *) DEFAULT_RTSP_PORT;
static int device_num = 10;
static int alert_port = 7552;

constexpr int FRAME_W = 640;
constexpr int FRAME_H = 480;

static GOptionEntry entries[] = {
    {"port", 'p', 0, G_OPTION_ARG_STRING, &port,
        "Port to listen on (default: " DEFAULT_RTSP_PORT ")", "PORT"},
    {"device", 'd', 0, G_OPTION_ARG_INT, &device_num,
        "Camera device number (default: 10 → /dev/video42)", "NUM"},
    {"alert-port", 'a', 0, G_OPTION_ARG_INT, &alert_port,
        "Alert server TCP port (default: 7552)", "PORT"},
    {NULL}
};

struct Client {
    GstElement *appsrc;
};
std::list<Client> g_clients;
std::mutex g_clients_mtx;

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data);
static void media_configure_cb(GstRTSPMediaFactory *factory, GstRTSPMedia *media, gpointer user_data);
static void media_finished_cb(GstRTSPMedia *media, gpointer user_data);
std::string get_local_ip();

rknn_app_context_t g_rknn_ctx;
object_detect_result_list g_od_results;
std::atomic<bool> g_running{true};
std::atomic<bool> g_got_frame{false};

int g_skeleton[38] = {16,14, 14,12, 17,15, 15,13, 12,13, 6,12, 7,13, 6,7,
                       6,8, 7,9, 8,10, 9,11, 2,3, 1,2, 1,3, 2,4, 3,5, 4,6, 5,7};

struct Recorder {
    GstElement *pipeline = nullptr;
    GstElement *src = nullptr;
    int frame_idx = 0;
    bool is_auto = false;
    std::string filename;

    bool init(const std::string &path) {
        std::string desc = "appsrc name=recsrc ! queue ! videoconvert ! mpph264enc "
                           "! h264parse ! mp4mux ! filesink location=\"" + path + "\"";
        pipeline = gst_parse_launch(desc.c_str(), nullptr);
        if (!pipeline) return false;
        src = gst_bin_get_by_name(GST_BIN(pipeline), "recsrc");
        if (!src) { gst_object_unref(pipeline); pipeline = nullptr; return false; }
        GstCaps *rc = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, "NV12",
            "width", G_TYPE_INT, FRAME_W,
            "height", G_TYPE_INT, FRAME_H,
            "framerate", GST_TYPE_FRACTION, 30, 1, NULL);
        gst_app_src_set_caps(GST_APP_SRC(src), rc);
        gst_caps_unref(rc);
        gst_app_src_set_stream_type(GST_APP_SRC(src), GST_APP_STREAM_TYPE_STREAM);
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        filename = path;
        return true;
    }

    void finish() {
        if (!pipeline) return;
        if (src) {
            gst_app_src_end_of_stream(GST_APP_SRC(src));
            GstBus *bus = gst_element_get_bus(pipeline);
            gst_bus_timed_pop_filtered(bus, 3 * GST_SECOND,
                static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
            gst_object_unref(bus);
        }
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(src);
        gst_object_unref(pipeline);
        pipeline = nullptr;
        src = nullptr;
    }
};

static void media_configure_cb(GstRTSPMediaFactory *factory, GstRTSPMedia *media, gpointer user_data)
{
    GstElement *pipeline = gst_rtsp_media_get_element(media);
    GstElement *appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "source");

    GstCaps *caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "NV12",
        "width", G_TYPE_INT, FRAME_W,
        "height", G_TYPE_INT, FRAME_H,
        NULL);
    gst_app_src_set_caps(GST_APP_SRC(appsrc), caps);
    gst_caps_unref(caps);

    gst_object_ref(appsrc);
    g_object_set_data(G_OBJECT(media), "appsrc", appsrc);

    std::lock_guard<std::mutex> lock(g_clients_mtx);
    g_clients.push_back({appsrc});
    std::cout << "RTSP client connected, total=" << g_clients.size() << std::endl;
}

static void media_finished_cb(GstRTSPMedia *media, gpointer user_data)
{
    GstElement *appsrc = (GstElement *)g_object_get_data(G_OBJECT(media), "appsrc");
    if (!appsrc) return;
    std::lock_guard<std::mutex> lock(g_clients_mtx);
    g_clients.remove_if([&](const Client &c) { return c.appsrc == appsrc; });
    gst_object_unref(appsrc);
    std::cout << "RTSP client disconnected, remaining=" << g_clients.size() << std::endl;
}

int main(int argc, char *argv[])
{
    GOptionContext *optctx;
    GError *error = NULL;

    optctx = g_option_context_new("- RTSP Camera Server");
    g_option_context_add_main_entries(optctx, entries, NULL);
    g_option_context_add_group(optctx, gst_init_get_option_group());
    if (!g_option_context_parse(optctx, &argc, &argv, &error)) {
        std::cerr << "Error parsing options: " << error->message << std::endl;
        g_option_context_free(optctx);
        g_clear_error(&error);
        return -1;
    }
    g_option_context_free(optctx);

    const char *model_path = argc > 1 ? argv[1] : "model/yolov8_pose.rknn";
    memset(&g_rknn_ctx, 0, sizeof(rknn_app_context_t));
    init_post_process();

    int init_ret = init_yolov8_pose_model(model_path, &g_rknn_ctx);
    if (init_ret != 0) {
        std::cerr << "Failed to init RKNN model: " << model_path << std::endl;
        return 1;
    }
    std::cout << "RKNN model loaded: " << model_path << std::endl;

    const std::string camera_path = "/dev/video" + std::to_string(device_num);
    const std::string IP = get_local_ip();
    const std::string W = std::to_string(FRAME_W);
    const std::string H = std::to_string(FRAME_H);

    if (access(camera_path.c_str(), R_OK) != 0) {
        std::cerr << "ERROR: " << camera_path << " not accessible!" << std::endl;
        return 1;
    }

    // ── 管道 1: 摄像头采集 → 缩放到 640x480（tee 分流，零拷贝）──
    const std::string cam_pipe =
        "v4l2src device=" + camera_path + " ! "
        "videoconvert ! videoscale ! "
        "video/x-raw,format=NV12,width=" + W + ",height=" + H + " ! "
        "tee name=t "
        "t. ! queue ! videoconvert ! video/x-raw,format=NV12 ! "
        "    appsink name=sink_ai max-buffers=1 drop=true "
        "t. ! queue ! videoconvert ! video/x-raw,format=NV12 ! "
        "    appsink name=sink_rtsp max-buffers=1 drop=true";

    std::cout << "Camera pipeline:\n" << cam_pipe << std::endl;

    GError *err = nullptr;
    GstElement *cam_pipeline = gst_parse_launch(cam_pipe.c_str(), &err);
    if (!cam_pipeline) {
        std::cerr << "Failed to create camera pipeline: " << err->message << std::endl;
        g_error_free(err);
        return 1;
    }
    std::cout << "Camera pipeline created OK" << std::endl;

    GstElement *sink_ai = gst_bin_get_by_name(GST_BIN(cam_pipeline), "sink_ai");
    GstElement *  sink_rtsp = gst_bin_get_by_name(GST_BIN(cam_pipeline), "sink_rtsp");
    if (!sink_ai) std::cerr << "WARN: sink_ai not found!" << std::endl;
    if (!sink_rtsp) std::cerr << "WARN: sink_rtsp not found!" << std::endl;

    // ── 管道 2: 本地显示 ──
    const std::string display_pipe =
        "appsrc name=local_src is-live=true format=time ! "
        "video/x-raw,format=NV12,width=" + W + ",height=" + H + " ! "
        "videoconvert ! "
        "autovideosink sync=false";

    std::cout << "Display pipeline:\n" << display_pipe << std::endl;

    GstElement *display_pipeline = gst_parse_launch(display_pipe.c_str(), &err);
    if (!display_pipeline) {
        std::cerr << "Failed to create display pipeline: " << err->message << std::endl;
        g_error_free(err);
        return 1;
    }
    std::cout << "Display pipeline created OK" << std::endl;

    GstElement *local_src = gst_bin_get_by_name(GST_BIN(display_pipeline), "local_src");
    if (!local_src) std::cerr << "WARN: local_src not found!" << std::endl;

    GstCaps *caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "NV12",
        "width", G_TYPE_INT, FRAME_W,
        "height", G_TYPE_INT, FRAME_H,
        NULL);
    gst_app_src_set_caps(GST_APP_SRC(local_src), caps);
    gst_caps_unref(caps);

    // ── 主循环 & bus ──
    auto loop = g_main_loop_new(nullptr, FALSE);

    auto bus1 = gst_pipeline_get_bus(GST_PIPELINE(cam_pipeline));
    gst_bus_add_watch(bus1, bus_call, loop);
    gst_object_unref(bus1);

    auto bus2 = gst_pipeline_get_bus(GST_PIPELINE(display_pipeline));
    gst_bus_add_watch(bus2, bus_call, loop);
    gst_object_unref(bus2);

    // ── 启动两个管道 ──
    GstStateChangeReturn r1 = gst_element_set_state(cam_pipeline, GST_STATE_PLAYING);
    if (r1 == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "ERROR: camera pipeline failed to PLAY" << std::endl;
        return 1;
    }
    std::cout << "Camera pipeline set to PLAYING (" << r1 << ") - device: " << camera_path << std::endl;

    GstStateChangeReturn r2 = gst_element_set_state(display_pipeline, GST_STATE_PLAYING);
    if (r2 == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "ERROR: display pipeline failed to PLAY" << std::endl;
        return 1;
    }
    std::cout << "Display pipeline set to PLAYING (" << r2 << ")" << std::endl;

    // ── 看门狗 ── 按引用捕获变量
    std::thread([&]() {
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::seconds(3));
            if (!g_got_frame)
                std::cout << "⚠ waiting for camera frames..." << std::endl;
        }
    }).detach();

    // ── RTSP 服务器 ──
    GstRTSPServer *server = gst_rtsp_server_new();
    g_object_set(server, "service", port, NULL);

    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

    std::string launch =
        "( appsrc name=source is-live=true format=time ! "
        "video/x-raw,format=NV12,width=" + W + ",height=" + H + " ! "
        "videoconvert ! video/x-raw,format=I420 ! "
        "mpph264enc ! "
        "rtph264pay name=pay0 pt=96 )";

    gst_rtsp_media_factory_set_launch(factory, launch.c_str());
    g_signal_connect(factory, "media-configure", G_CALLBACK(media_configure_cb), NULL);

    gst_rtsp_mount_points_add_factory(mounts, "/test", factory);
    g_object_unref(mounts);

    gst_rtsp_server_attach(server, NULL);
    std::cout << "RTSP stream ready at rtsp://" << IP << ":" << port << "/test" << std::endl;

    // ── RTSP 推流线程 ──
    std::thread([sink_rtsp]() {
        int rtsp_frames = 0;
        auto rtsp_last = std::chrono::steady_clock::now();

        while (g_running) {
            GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_rtsp));
            if (!sample) { std::cerr << "rtsp: no sample" << std::endl; break; }

            g_got_frame = true;
            GstBuffer *buf = gst_sample_get_buffer(sample);

            std::lock_guard<std::mutex> lock(g_clients_mtx);
            for (auto it = g_clients.begin(); it != g_clients.end(); ) {
                GstFlowReturn r = gst_app_src_push_buffer(GST_APP_SRC(it->appsrc), gst_buffer_ref(buf));
                if (r != GST_FLOW_OK) it = g_clients.erase(it);
                else ++it;
            }

            gst_sample_unref(sample);
            rtsp_frames++;

            auto now = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - rtsp_last).count() / 1000.f;
            if (elapsed >= 5.0f) {
                std::cout << "RTSP: " << (int)(rtsp_frames / elapsed) << " fps | clients=" << g_clients.size() << std::endl;
                rtsp_frames = 0;
                rtsp_last = now;
            }
        }
    }).detach();

    // ── 告警 TCP 服务器 ──
    AlertServer alert_server(alert_port);
    if (!alert_server.start("alert.key")) {
        std::cout << "  (cam will run without alert functionality)" << std::endl;
    }

    // ── AI 推理 + 本地显示线程 ──
    std::thread([sink_ai, local_src, &alert_server]() {
        int frame_count = 0;
        auto last_time = std::chrono::steady_clock::now();

        std::vector<Recorder> recorders;

        enum AutoRecState : int { AUTO_IDLE, AUTO_WARMING, AUTO_ACTIVE };
        AutoRecState auto_state = AUTO_IDLE;
        auto warmup_start = std::chrono::steady_clock::time_point{};
        auto auto_rec_start = std::chrono::steady_clock::time_point{};
        int normal_count = 0;
        bool prev_abnormal = false;

        while (g_running) {
            GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_ai));
            if (!sample) { std::cerr << "ai: no sample" << std::endl; break; }

            g_got_frame = true;

            // ── 获取 caps（优先从 sample，fallback 到 appsink pad）──
            GstCaps *caps = gst_sample_get_caps(sample);
            if (!caps) {
                GstPad *pad = gst_element_get_static_pad(GST_ELEMENT(sink_ai), "sink");
                if (pad) {
                    caps = gst_pad_get_current_caps(pad);
                    gst_object_unref(pad);
                }
            }

            int w = FRAME_W, h = FRAME_H;
            std::string fmt_str = "?";
            if (caps) {
                GstStructure *s = gst_caps_get_structure(caps, 0);
                if (s) {
                    gst_structure_get_int(s, "width", &w);
                    gst_structure_get_int(s, "height", &h);
                    const char *f = gst_structure_get_string(s, "format");
                    if (f) fmt_str = f;
                }
                gst_caps_unref(caps);
            }

            static bool first = true;
            if (first) {
                first = false;
                std::cout << "▲ first frame: " << w << "x" << h << " " << fmt_str << std::endl;
            }

            auto t1 = std::chrono::steady_clock::now();

            // ── Deep-copy buffer: tee 共享内存不可写，我们必须有自己的副本才能画框 ──
            GstBuffer *orig = gst_sample_get_buffer(sample);

            GstMapInfo src_info;
            gst_buffer_map(orig, &src_info, GST_MAP_READ);
            GstBuffer *buf = gst_buffer_new_and_alloc(src_info.size);
            GstMapInfo dst_info;
            gst_buffer_map(buf, &dst_info, GST_MAP_WRITE);
            memcpy(dst_info.data, src_info.data, src_info.size);
            gst_buffer_unmap(buf, &dst_info);
            gst_buffer_unmap(orig, &src_info);

            GST_BUFFER_PTS(buf) = GST_BUFFER_PTS(orig);
            GST_BUFFER_DTS(buf) = GST_BUFFER_DTS(orig);
            GST_BUFFER_DURATION(buf) = GST_BUFFER_DURATION(orig);

            gst_sample_unref(sample); // 释放原始 buffer，不影响 RTSP 那边

            // ── map 副本画图 ──
            GstMapInfo info;
            gst_buffer_map(buf, &info, (GstMapFlags)(GST_MAP_READ | GST_MAP_WRITE));

            image_buffer_t src_img;
            memset(&src_img, 0, sizeof(image_buffer_t));
            src_img.width = w;
            src_img.height = h;
            src_img.width_stride = w;
            src_img.height_stride = h;
            src_img.format = IMAGE_FORMAT_YUV420SP_NV12;
            src_img.virt_addr = info.data;
            src_img.size = info.size;

            int ret = -1;
            static bool skip_ai = false;

            if (!skip_ai) {
                ret = inference_yolov8_pose_model(&g_rknn_ctx, &src_img, &g_od_results);
                if (ret != 0) {
                    std::cerr << "AI inference failed, skipping" << std::endl;
                    skip_ai = true;
                }
            }

            auto t2 = std::chrono::steady_clock::now();
            int infer_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

            if (ret == 0 && !skip_ai) {
                char text[256];
                for (int i = 0; i < g_od_results.count; i++) {
                    object_detect_result *det = &g_od_results.results[i];
                    int x1 = det->box.left, y1 = det->box.top;
                    int x2 = det->box.right, y2 = det->box.bottom;

                    draw_rectangle(&src_img, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);
                    sprintf(text, "person %.1f%%", det->prop * 100);
                    draw_text(&src_img, text, x1, y1 - 20, COLOR_RED, 10);

                    for (int j = 0; j < 19; j++) {
                        int idx1 = g_skeleton[2*j] - 1;
                        int idx2 = g_skeleton[2*j+1] - 1;
                        draw_line(&src_img,
                            (int)det->keypoints[idx1][0], (int)det->keypoints[idx1][1],
                            (int)det->keypoints[idx2][0], (int)det->keypoints[idx2][1],
                            COLOR_ORANGE, 3);
                    }
                    for (int j = 0; j < 17; j++) {
                        draw_circle(&src_img,
                            (int)det->keypoints[j][0], (int)det->keypoints[j][1],
                            3, COLOR_YELLOW, 2);
                    }
                }

                // ── 行为分析 ──
                BehaviorReport br;
                memset(&br, 0, sizeof(br));
                br = analyze_behavior(g_od_results, 4);
                int line_y = 30;
                char text_buf[128];
                sprintf(text_buf, "people=%d", br.people_count);
                draw_text(&src_img, text_buf, 10, line_y, COLOR_GREEN, 10);
                line_y += 20;
                if (br.crowd_alert) {
                    draw_text(&src_img, "CROWD ALERT", 10, line_y, COLOR_RED, 12);
                    line_y += 20;
                }
                if (br.crouching) {
                    draw_text(&src_img, "CROUCH", 10, line_y, COLOR_YELLOW, 10);
                    line_y += 20;
                }
                if (br.violence_alert) {
                    draw_text(&src_img, "VIOLENCE", 10, line_y, COLOR_RED, 12);
                    line_y += 20;
                }

                // ── 告警推送 + 命令处理 + 自动录制 ──
                bool abnormal = br.violence_alert || br.crouching || br.crowd_alert;

                // ── 告警推送（状态变化时发送）──
                if (alert_server.has_clients()) {
                    if (abnormal && !prev_abnormal) {
                        char buf[256];
                        std::string types;
                        if (br.crowd_alert) types += "\"crowd\",";
                        if (br.crouching) types += "\"crouch\",";
                        if (br.violence_alert) types += "\"violence\",";
                        if (!types.empty()) types.pop_back();
                        snprintf(buf, sizeof(buf),
                            "{\"event\":\"alert\",\"types\":[%s],\"people\":%d}",
                            types.c_str(), br.people_count);
                        alert_server.send_msg(buf);
                    } else if (!abnormal && prev_abnormal) {
                        alert_server.send_msg("{\"event\":\"normal\"}");
                    }
                }
                prev_abnormal = abnormal;

                // ── 命令处理 ──
                {
                    AlertCommand cmd;
                    while (alert_server.pop_command(cmd)) {
                        if (cmd.cmd == "start_record") {
                            auto now_t = std::chrono::system_clock::now();
                            auto tt = std::chrono::system_clock::to_time_t(now_t);
                            char path[128];
                            strftime(path, sizeof(path), "cmd_%Y%m%d_%H%M%S.mp4", localtime(&tt));
                            Recorder r;
                            r.is_auto = false;
                            if (r.init(path)) {
                                recorders.push_back(std::move(r));
                                std::cout << "[CMD] recording started: " << path << std::endl;
                            }
                        } else if (cmd.cmd == "stop_record") {
                            for (int i = (int)recorders.size() - 1; i >= 0; i--) {
                                if (!recorders[i].is_auto) {
                                    std::cout << "[CMD] recording stopped: " << recorders[i].filename << std::endl;
                                    recorders[i].finish();
                                    recorders.erase(recorders.begin() + i);
                                    break;
                                }
                            }
                        } else if (cmd.cmd == "get_status") {
                            char buf[128];
                            snprintf(buf, sizeof(buf),
                                "{\"event\":\"status\",\"recorders\":%zu}", recorders.size());
                            alert_server.send_msg(buf);
                        }
                    }
                }

                // ── 自动录制状态机（仅异常触发）──
                if (abnormal) {
                    normal_count = 0;
                } else {
                    normal_count++;
                }

                switch (auto_state) {
                case AUTO_IDLE:
                    if (abnormal) {
                        auto_state = AUTO_WARMING;
                        warmup_start = std::chrono::steady_clock::now();
                    }
                    break;
                case AUTO_WARMING:
                    if (normal_count >= 30) {
                        auto_state = AUTO_IDLE;
                    } else if (abnormal) {
                        auto elapsed = std::chrono::steady_clock::now() - warmup_start;
                        if (elapsed >= std::chrono::seconds(3)) {
                            auto now_t = std::chrono::system_clock::now();
                            auto tt = std::chrono::system_clock::to_time_t(now_t);
                            char path[128];
                            strftime(path, sizeof(path), "record_%Y%m%d_%H%M%S.mp4", localtime(&tt));
                            Recorder r;
                            r.is_auto = true;
                            if (r.init(path)) {
                                recorders.push_back(std::move(r));
                                std::cout << "[REC] auto started: " << path << std::endl;
                            }
                            auto_state = AUTO_ACTIVE;
                            auto_rec_start = std::chrono::steady_clock::now();
                        }
                    }
                    break;
                case AUTO_ACTIVE:
                    if (normal_count >= 60) {
                        auto rec_elapsed = std::chrono::steady_clock::now() - auto_rec_start;
                        if (rec_elapsed >= std::chrono::seconds(5)) {
                            for (auto it = recorders.begin(); it != recorders.end(); ) {
                                if (it->is_auto) {
                                    std::cout << "[REC] auto stopped" << std::endl;
                                    it->finish();
                                    it = recorders.erase(it);
                                } else {
                                    ++it;
                                }
                            }
                            auto_state = AUTO_IDLE;
                        } else {
                            std::cout << "[REC] safety hold: "
                                      << std::chrono::duration_cast<std::chrono::seconds>(rec_elapsed).count()
                                      << "s" << std::endl;
                        }
                    }
                    break;
                }

                // ── 录制指示器 ──
                if (!recorders.empty()) {
                    draw_text(&src_img, "REC", FRAME_W - 60, 20, COLOR_RED, 15);
                }
            } // end if ret==0

            gst_buffer_unmap(buf, &info);

            // ── 投递到各路录制管道 ──
            for (auto &r : recorders) {
                GstBuffer *rec_copy = gst_buffer_copy(buf);
                GST_BUFFER_PTS(rec_copy) = gst_util_uint64_scale(r.frame_idx++, GST_SECOND, 30);
                GST_BUFFER_DURATION(rec_copy) = gst_util_uint64_scale(1, GST_SECOND, 30);
                gst_app_src_push_buffer(GST_APP_SRC(r.src), rec_copy);
            }

            // ── 投递到本地显示 ──
            gst_app_src_push_buffer(GST_APP_SRC(local_src), buf);

            frame_count++;
            auto now = std::chrono::steady_clock::now();
            float elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count() / 1000.f;
            if (elapsed_s >= 5.0f) {
                int fps = (int)(frame_count / elapsed_s);
                std::cout << "AI: " << w << "x" << h
                          << " | infer=" << infer_ms << "ms"
                          << " | det=" << g_od_results.count
                          << " | FPS=" << fps << std::endl;
                frame_count = 0;
                last_time = now;
            }
        }
    }).detach();

    g_main_loop_run(loop);

    g_running = false;
    alert_server.stop();
    gst_element_set_state(cam_pipeline, GST_STATE_NULL);
    gst_element_set_state(display_pipeline, GST_STATE_NULL);
    gst_object_unref(cam_pipeline);
    gst_object_unref(display_pipeline);
    g_main_loop_unref(loop);

    release_yolov8_pose_model(&g_rknn_ctx);
    deinit_post_process();
}

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
    auto loop = static_cast<GMainLoop *>(data);

    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS:
            std::cout << "BUS: EOS" << std::endl;
            g_main_loop_quit(loop);
            break;
        case GST_MESSAGE_ERROR: {
            gchar *debug;
            GError *err;
            gst_message_parse_error(msg, &err, &debug);
            const gchar *name = GST_OBJECT_NAME(GST_MESSAGE_SRC(msg));
            std::cerr << "BUS ERROR [" << (name ? name : "?") << "]: " << err->message << std::endl;
            if (debug) std::cerr << "  debug: " << debug << std::endl;
            g_error_free(err);
            g_free(debug);
            g_main_loop_quit(loop);
            break;
        }
        case GST_MESSAGE_WARNING: {
            gchar *debug;
            GError *err;
            gst_message_parse_warning(msg, &err, &debug);
            const gchar *name = GST_OBJECT_NAME(GST_MESSAGE_SRC(msg));
            std::cerr << "BUS WARNING [" << (name ? name : "?") << "]: " << err->message << std::endl;
            if (debug) std::cerr << "  debug: " << debug << std::endl;
            g_error_free(err);
            g_free(debug);
            break;
        }
        case GST_MESSAGE_STATE_CHANGED: {
            if (GST_IS_PIPELINE(GST_MESSAGE_SRC(msg))) {
                GstState old, new_state, pending;
                gst_message_parse_state_changed(msg, &old, &new_state, &pending);
                const gchar *name = GST_OBJECT_NAME(GST_MESSAGE_SRC(msg));
                std::cout << "BUS [" << name << "] state: "
                    << gst_element_state_get_name(old) << " → "
                    << gst_element_state_get_name(new_state) << std::endl;
            }
            break;
        }
        default:
            break;
    }
    return TRUE;
}

std::string get_local_ip()
{
    struct ifaddrs *ifaddr, *ifa;
    std::string ip;
    getifaddrs(&ifaddr);
    for (ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET)
            continue;
        if (strcmp(ifa->ifa_name, "lo") == 0)
            continue;
        ip = inet_ntoa(((struct sockaddr_in*)ifa->ifa_addr)->sin_addr);
        break;
    }
    freeifaddrs(ifaddr);
    return ip;
}

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <iostream>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstring>
#include <string>
#include <thread>
#include <mutex>
#include <list>

#include "yolov8-pose.h"
#include "image_utils.h"
#include "image_drawing.h"

#define DEFAULT_RTSP_PORT "7551"

static char *port = (char *) DEFAULT_RTSP_PORT;

static GOptionEntry entries[] = {
    {"port", 'p', 0, G_OPTION_ARG_STRING, &port,
        "Port to listen on (default: " DEFAULT_RTSP_PORT ")", "PORT"},
    {NULL}
};

struct Client {
    GstElement *appsrc;
};

std::list<Client> g_clients;
std::mutex g_clients_mtx;
bool g_running = true;

rknn_app_context_t g_rknn_ctx;
object_detect_result_list g_od_results;

// COCO skeleton connections for drawing
int g_skeleton[38] = {16,14, 14,12, 17,15, 15,13, 12,13, 6,12, 7,13, 6,7,
                       6,8, 7,9, 8,10, 9,11, 2,3, 1,2, 1,3, 2,4, 3,5, 4,6, 5,7};

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data);
static void media_configure_cb(GstRTSPMediaFactory *factory, GstRTSPMedia *media, gpointer user_data);
static void media_finished_cb(GstRTSPMedia *media, gpointer user_data);
std::string get_local_ip();

static void media_configure_cb(GstRTSPMediaFactory *factory, GstRTSPMedia *media, gpointer user_data)
{
    GstElement *pipeline = gst_rtsp_media_get_element(media);
    GstElement *appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "source");

    GstCaps *caps = gst_caps_new_simple("video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        "width", G_TYPE_INT, 640,
        "height", G_TYPE_INT, 480,
        NULL);
    gst_app_src_set_caps(GST_APP_SRC(appsrc), caps);
    gst_caps_unref(caps);

    gst_object_ref(appsrc);
    g_object_set_data(G_OBJECT(media), "appsrc", appsrc);

    std::lock_guard<std::mutex> lock(g_clients_mtx);
    g_clients.push_back({appsrc});
}

static void media_finished_cb(GstRTSPMedia *media, gpointer user_data)
{
    GstElement *appsrc = (GstElement *)g_object_get_data(G_OBJECT(media), "appsrc");
    if (!appsrc) return;

    std::lock_guard<std::mutex> lock(g_clients_mtx);
    g_clients.remove_if([&](const Client &c) { return c.appsrc == appsrc; });
    gst_object_unref(appsrc);
}

static void pull_and_push_frames(GstElement *sink)
{
    while (g_running) {
        GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
        if (!sample) break;

        GstBuffer *buf = gst_sample_get_buffer(sample);
        GstCaps *caps = gst_sample_get_caps(sample);
        if (!caps) { gst_sample_unref(sample); continue; }

        GstStructure *s = gst_caps_get_structure(caps, 0);
        int w, h;
        gst_structure_get_int(s, "width", &w);
        gst_structure_get_int(s, "height", &h);
        gst_caps_unref(caps);

        GstMapInfo info;
        gst_buffer_map(buf, &info, (GstMapFlags)(GST_MAP_READ | GST_MAP_WRITE));

        // ── RKNN 推理 ──
        image_buffer_t src_img;
        memset(&src_img, 0, sizeof(image_buffer_t));
        src_img.width = w;
        src_img.height = h;
        src_img.width_stride = w * 3;
        src_img.height_stride = h;
        src_img.format = IMAGE_FORMAT_RGB888;   // 数据实际是 BGR，推理不受影响
        src_img.virt_addr = info.data;
        src_img.size = info.size;

        int ret = inference_yolov8_pose_model(&g_rknn_ctx, &src_img, &g_od_results);
        if (ret == 0) {
            // ── 画检测结果（直接在原图 BGR 数据上画） ──
            char text[256];
            for (int i = 0; i < g_od_results.count; i++) {
                object_detect_result *det = &g_od_results.results[i];
                int x1 = det->box.left, y1 = det->box.top;
                int x2 = det->box.right, y2 = det->box.bottom;

                draw_rectangle(&src_img, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);
                sprintf(text, "person %.1f%%", det->prop * 100);
                draw_text(&src_img, text, x1, y1 - 20, COLOR_RED, 10);

                // 骨架
                for (int j = 0; j < 19; j++) {
                    int idx1 = g_skeleton[2*j] - 1;
                    int idx2 = g_skeleton[2*j+1] - 1;
                    draw_line(&src_img,
                        (int)det->keypoints[idx1][0], (int)det->keypoints[idx1][1],
                        (int)det->keypoints[idx2][0], (int)det->keypoints[idx2][1],
                        COLOR_ORANGE, 3);
                }
                // 关键点
                for (int j = 0; j < 17; j++) {
                    draw_circle(&src_img,
                        (int)det->keypoints[j][0], (int)det->keypoints[j][1],
                        3, COLOR_YELLOW, 2);
                }
            }
        } else {
            static int once = 0;
            if (!once) { std::cerr << "inference fail: " << ret << std::endl; once = 1; }
        }

        gst_buffer_unmap(buf, &info);

        // ── 推给所有 RTSP 客户端 ──
        std::lock_guard<std::mutex> lock(g_clients_mtx);
        for (auto it = g_clients.begin(); it != g_clients.end(); ) {
            GstFlowReturn r = gst_app_src_push_buffer(GST_APP_SRC(it->appsrc), gst_buffer_ref(buf));
            if (r != GST_FLOW_OK) {
                it = g_clients.erase(it);
                continue;
            }
            ++it;
        }

        gst_sample_unref(sample);
    }
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

    // ── 加载 RKNN 模型 ──
    const char *model_path = argc > 1 ? argv[1] : "model/yolov8n-pose.rknn";
    memset(&g_rknn_ctx, 0, sizeof(rknn_app_context_t));
    init_post_process();

    int init_ret = init_yolov8_pose_model(model_path, &g_rknn_ctx);
    if (init_ret != 0) {
        std::cerr << "Failed to init RKNN model: " << model_path << std::endl;
        return 1;
    }
    std::cout << "RKNN model loaded: " << model_path << std::endl;

    const std::string camera_path = "/dev/video10";
    const std::string IP = get_local_ip();

    const std::string local_pipe =
        "v4l2src device=" + camera_path + " ! "
        "videoconvert ! "
        "tee name=t "
        "t. ! queue ! autovideosink "
        "t. ! queue ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=sink max-buffers=1 drop=true";

    GError *err = nullptr;
    GstElement *pipeline = gst_parse_launch(local_pipe.c_str(), &err);
    if (!pipeline) {
        std::cerr << "Failed to create pipeline: " << err->message << std::endl;
        g_error_free(err);
        return 1;
    }
    GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");

    auto loop = g_main_loop_new(nullptr, FALSE);
    auto bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    gst_bus_add_watch(bus, bus_call, loop);
    gst_object_unref(bus);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    std::cout << "Camera open at " << camera_path << std::endl;

    // ── RTSP 服务器 ──
    GstRTSPServer *server = gst_rtsp_server_new();
    g_object_set(server, "service", port, NULL);

    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

    std::string launch =
        "( appsrc name=source is-live=true format=time ! "
        "video/x-raw,format=BGR,width=640,height=480 ! "
        "videoconvert ! video/x-raw,format=I420 ! "
        "x264enc tune=zerolatency bitrate=2000 ! "
        "rtph264pay name=pay0 pt=96 )";

    gst_rtsp_media_factory_set_launch(factory, launch.c_str());
    g_signal_connect(factory, "media-configure", G_CALLBACK(media_configure_cb), NULL);

    gst_rtsp_mount_points_add_factory(mounts, "/test", factory);
    g_object_unref(mounts);

    gst_rtsp_server_attach(server, NULL);
    std::cout << "RTSP stream ready at rtsp://" << IP << ":" << port << "/test" << std::endl;

    std::thread(pull_and_push_frames, sink).detach();

    g_main_loop_run(loop);

    g_running = false;
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);

    release_yolov8_pose_model(&g_rknn_ctx);
    deinit_post_process();
}

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
    auto loop = static_cast<GMainLoop *>(data);
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS:
            g_main_loop_quit(loop);
            break;
        case GST_MESSAGE_ERROR:
            gchar *debug;
            GError *err;
            gst_message_parse_error(msg, &err, &debug);
            std::cerr << "Error: " << err->message << std::endl;
            std::cerr << "Debug: " << debug << std::endl;
            g_error_free(err);
            g_free(debug);
            g_main_loop_quit(loop);
            break;
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

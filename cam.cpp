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

        std::lock_guard<std::mutex> lock(g_clients_mtx);
        for (auto it = g_clients.begin(); it != g_clients.end(); ) {
            GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(it->appsrc), gst_buffer_ref(buf));
            if (ret != GST_FLOW_OK) {
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

    const std::string camera_path = "/dev/video10";
    const std::string IP = get_local_ip();

    // ── 本地 pipeline：只开一次摄像头 ──
    const std::string local_pipe =
        "v4l2src device=" + camera_path + " ! "
        "videoconvert ! "
        "tee name=t "
        "t. ! queue ! autovideosink "
        "t. ! queue ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=sink max-buffers=1 drop=true";

    std::cout << "Pipeline: " << local_pipe << std::endl;

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

    // ── RTSP 服务器：appsrc 从主 pipeline 取帧 ──
    GstRTSPServer *server = gst_rtsp_server_new();
    g_object_set(server, "service", port, NULL);

    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

    // 注意：外部用圆括号包起来，pay0 是 rtsp server 要求的 payloader 命名
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

    // ── 启动拉帧 + 推 RTSP 线程 ──
    std::thread(pull_and_push_frames, sink).detach();

    g_main_loop_run(loop);

    g_running = false;
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);
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

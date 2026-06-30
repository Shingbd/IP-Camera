#pragma once
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <queue>
#include "crypto_utils.h"

struct AlertCommand {
    std::string cmd;
    std::string args;
};

class AlertServer {
public:
    explicit AlertServer(int port = 7552);
    ~AlertServer();

    bool start(const char *key_path);
    void stop();

    void send_msg(const std::string &plaintext);
    bool pop_command(AlertCommand &cmd);
    bool has_clients() const;
    int client_count() const;

private:
    void accept_loop();
    void client_read_loop(int fd);
    bool recv_all(int fd, uint8_t *buf, size_t size);

    int m_port;
    int m_listen_fd;
    std::atomic<bool> m_running;
    std::thread m_accept_thread;
    unsigned char m_key[32];
    bool m_key_loaded;

    mutable std::mutex m_clients_mtx;
    std::vector<int> m_clients;

    std::mutex m_cmd_mtx;
    std::queue<AlertCommand> m_cmd_queue;
};

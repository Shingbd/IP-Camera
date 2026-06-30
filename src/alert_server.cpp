#include "alert_server.h"
#include <iostream>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

AlertServer::AlertServer(int port)
    : m_port(port)
    , m_listen_fd(-1)
    , m_running(false)
    , m_key_loaded(false)
{
    memset(m_key, 0, 32);
}

AlertServer::~AlertServer()
{
    stop();
}

bool AlertServer::start(const char *key_path)
{
    m_key_loaded = load_key_file(key_path, m_key);
    if (!m_key_loaded) {
        std::cerr << "[ALERT] no key file: " << key_path << std::endl;
        return false;
    }

    m_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_listen_fd < 0) {
        std::cerr << "[ALERT] socket() failed: " << strerror(errno) << std::endl;
        return false;
    }

    int opt = 1;
    setsockopt(m_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(m_port);

    if (bind(m_listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[ALERT] bind() port " << m_port << " failed: "
                  << strerror(errno) << std::endl;
        close(m_listen_fd);
        m_listen_fd = -1;
        return false;
    }

    if (listen(m_listen_fd, 5) < 0) {
        std::cerr << "[ALERT] listen() failed: " << strerror(errno) << std::endl;
        close(m_listen_fd);
        m_listen_fd = -1;
        return false;
    }

    m_running = true;
    m_accept_thread = std::thread(&AlertServer::accept_loop, this);
    std::cout << "[ALERT] TCP server listening on port " << m_port << std::endl;
    return true;
}

void AlertServer::stop()
{
    m_running = false;

    if (m_listen_fd >= 0) {
        close(m_listen_fd);
        m_listen_fd = -1;
    }

    {
        std::lock_guard<std::mutex> lock(m_clients_mtx);
        for (int fd : m_clients) close(fd);
        m_clients.clear();
    }

    if (m_accept_thread.joinable())
        m_accept_thread.join();
}

void AlertServer::send_msg(const std::string &plaintext)
{
    if (!m_key_loaded || m_clients.empty()) return;

    std::vector<uint8_t> cipher = encrypt_msg(m_key, plaintext);
    if (cipher.empty()) return;

    uint32_t len = cipher.size();
    uint8_t header[4];
    header[0] = (len >> 24) & 0xff;
    header[1] = (len >> 16) & 0xff;
    header[2] = (len >> 8) & 0xff;
    header[3] = len & 0xff;

    std::lock_guard<std::mutex> lock(m_clients_mtx);
    for (auto it = m_clients.begin(); it != m_clients.end(); ) {
        ssize_t n = send(*it, header, 4, MSG_NOSIGNAL);
        if (n == 4)
            n = send(*it, cipher.data(), cipher.size(), MSG_NOSIGNAL);
        if (n != static_cast<ssize_t>(cipher.size())) {
            close(*it);
            it = m_clients.erase(it);
            std::cout << "[ALERT] client disconnected (send)" << std::endl;
        } else {
            ++it;
        }
    }
}

bool AlertServer::pop_command(AlertCommand &cmd)
{
    std::lock_guard<std::mutex> lock(m_cmd_mtx);
    if (m_cmd_queue.empty()) return false;
    cmd = m_cmd_queue.front();
    m_cmd_queue.pop();
    return true;
}

bool AlertServer::has_clients() const
{
    std::lock_guard<std::mutex> lock(m_clients_mtx);
    return !m_clients.empty();
}

int AlertServer::client_count() const
{
    std::lock_guard<std::mutex> lock(m_clients_mtx);
    return m_clients.size();
}

bool AlertServer::recv_all(int fd, uint8_t *buf, size_t size)
{
    while (size > 0) {
        ssize_t n = recv(fd, buf, size, 0);
        if (n <= 0) return false;
        buf += n;
        size -= n;
    }
    return true;
}

void AlertServer::accept_loop()
{
    while (m_running) {
        struct sockaddr_in addr;
        socklen_t addrlen = sizeof(addr);
        int fd = accept(m_listen_fd, (struct sockaddr*)&addr, &addrlen);
        if (fd < 0) {
            if (errno == EINTR) continue;
            break;
        }

        char peer[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr.sin_addr, peer, sizeof(peer));
        std::cout << "[ALERT] client connected: " << peer
                  << ":" << ntohs(addr.sin_port) << std::endl;

        {
            std::lock_guard<std::mutex> lock(m_clients_mtx);
            m_clients.push_back(fd);
        }

        std::thread(&AlertServer::client_read_loop, this, fd).detach();
    }
}

void AlertServer::client_read_loop(int fd)
{
    uint8_t header[4];
    while (m_running) {
        if (!recv_all(fd, header, 4)) break;

        uint32_t msg_len = (header[0] << 24) | (header[1] << 16)
                         | (header[2] << 8) | header[3];
        if (msg_len == 0 || msg_len > 1024 * 1024) break;

        std::vector<uint8_t> cipher(msg_len);
        if (!recv_all(fd, cipher.data(), msg_len)) break;

        std::string plain = decrypt_msg(m_key, cipher.data(), cipher.size());
        if (plain.empty()) {
            std::cerr << "[ALERT] decrypt/ auth failed" << std::endl;
            continue;
        }

        AlertCommand cmd;
        if (plain.find("\"start_record\"") != std::string::npos) {
            cmd.cmd = "start_record";
        } else if (plain.find("\"stop_record\"") != std::string::npos) {
            cmd.cmd = "stop_record";
        } else if (plain.find("\"get_status\"") != std::string::npos) {
            cmd.cmd = "get_status";
        } else if (plain.find("\"sentinel_mode\"") != std::string::npos) {
            cmd.cmd = "sentinel_mode";
            cmd.args = plain;
        } else {
            std::cerr << "[ALERT] unknown command: " << plain << std::endl;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(m_cmd_mtx);
            m_cmd_queue.push(cmd);
        }
    }

    close(fd);
    {
        std::lock_guard<std::mutex> lock(m_clients_mtx);
        auto it = std::find(m_clients.begin(), m_clients.end(), fd);
        if (it != m_clients.end()) m_clients.erase(it);
    }
    std::cout << "[ALERT] client disconnected (read)" << std::endl;
}

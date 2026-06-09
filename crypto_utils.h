#pragma once
#include <vector>
#include <string>
#include <cstdint>

bool load_key_file(const char *path, unsigned char key[32]);
std::vector<uint8_t> encrypt_msg(const unsigned char key[32], const std::string &plain);
std::string decrypt_msg(const unsigned char key[32], const uint8_t *data, size_t len);

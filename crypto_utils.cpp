#include "crypto_utils.h"
#include <fstream>
#include <cstring>
#include <openssl/evp.h>
#include <openssl/rand.h>

static const int IV_LEN = 12;
static const int TAG_LEN = 16;

bool load_key_file(const char *path, unsigned char key[32])
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    f.read(reinterpret_cast<char*>(key), 32);
    return f.gcount() == 32;
}

std::vector<uint8_t> encrypt_msg(const unsigned char key[32], const std::string &plain)
{
    std::vector<uint8_t> out;
    EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
    if (!ctx) return out;

    uint8_t iv[IV_LEN];
    RAND_bytes(iv, IV_LEN);

    EVP_EncryptInit_ex(ctx, EVP_aes_256_gcm(), nullptr, nullptr, nullptr);
    EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_IVLEN, IV_LEN, nullptr);
    EVP_EncryptInit_ex(ctx, nullptr, nullptr, key, iv);

    out.resize(IV_LEN + plain.size() + TAG_LEN);
    int len = 0, total = 0;
    EVP_EncryptUpdate(ctx, out.data() + IV_LEN, &len,
                      reinterpret_cast<const uint8_t*>(plain.data()), plain.size());
    total = len;
    EVP_EncryptFinal_ex(ctx, out.data() + IV_LEN + total, &len);
    total += len;
    EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_GET_TAG, TAG_LEN,
                         out.data() + IV_LEN + total);
    total += TAG_LEN;
    out.resize(IV_LEN + total);
    memcpy(out.data(), iv, IV_LEN);

    EVP_CIPHER_CTX_free(ctx);
    return out;
}

std::string decrypt_msg(const unsigned char key[32], const uint8_t *data, size_t len)
{
    if (len < IV_LEN + TAG_LEN) return {};

    EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
    if (!ctx) return {};

    const uint8_t *iv = data;
    const uint8_t *cipher = data + IV_LEN;
    size_t cipher_len = len - IV_LEN - TAG_LEN;
    const uint8_t *tag = data + IV_LEN + cipher_len;

    EVP_DecryptInit_ex(ctx, EVP_aes_256_gcm(), nullptr, nullptr, nullptr);
    EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_IVLEN, IV_LEN, nullptr);
    EVP_DecryptInit_ex(ctx, nullptr, nullptr, key, iv);

    std::string plain(cipher_len, '\0');
    int outlen = 0, total = 0;
    EVP_DecryptUpdate(ctx, reinterpret_cast<uint8_t*>(&plain[0]), &outlen,
                      cipher, cipher_len);
    total = outlen;

    EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_TAG, TAG_LEN, const_cast<uint8_t*>(tag));
    int ret = EVP_DecryptFinal_ex(ctx, reinterpret_cast<uint8_t*>(&plain[0]) + total, &outlen);
    EVP_CIPHER_CTX_free(ctx);

    if (ret <= 0) return {};
    total += outlen;
    plain.resize(total);
    return plain;
}

// NVSWrapper.cpp
#include "NVSWrapper.h"
#include <esp_log.h>
#include <cstring>

static const char* TAG = "NVSWrapper";

NVSWrapper::NVSWrapper(const std::string& namespace_name) : 
    m_namespace(namespace_name), m_handle(0) {
    if (!initializeNVS()) {
        throw std::runtime_error("Failed to initialize NVS");
    }
}

NVSWrapper::~NVSWrapper() {
    if (m_handle) {
        nvs_close(m_handle);
    }
}

bool NVSWrapper::initializeNVS() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Open namespace
    esp_err_t err = nvs_open(m_namespace.c_str(), NVS_READWRITE, &m_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

bool NVSWrapper::commit() {
    esp_err_t err = nvs_commit(m_handle);
    return handleError(err);
}

bool NVSWrapper::handleError(esp_err_t err, bool ignoreNotFound) {
    if (err == ESP_OK) {
        return true;
    }

    if (ignoreNotFound && err == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

bool NVSWrapper::writeString(const std::string& key, const std::string& value) {
    // Include null terminator
    if (!validateKey(key)) return false;
    return writeVector<char>(key, std::vector<char>(value.begin(), value.end()));
}

bool NVSWrapper::readString(const std::string& key, std::string& value) {
    std::vector<char> buffer;

    if (!validateKey(key)) return false;

    if (!readVector<char>(key, buffer) || buffer.empty()) {
        value.clear();
        return false;
    }
    value.assign(buffer.begin(), buffer.end());
    return true;
}

bool NVSWrapper::keyExists(const std::string& key) {
    size_t required_size = 0;
    esp_err_t err = nvs_get_blob(m_handle, key.c_str(), nullptr, &required_size);
    return err == ESP_OK || err == ESP_ERR_NVS_INVALID_LENGTH;
}

bool NVSWrapper::removeKey(const std::string& key) {
    esp_err_t err = nvs_erase_key(m_handle, key.c_str());
    return handleError(err) && commit();
}

bool NVSWrapper::eraseAll() {
    esp_err_t err = nvs_erase_all(m_handle);
    return handleError(err) && commit();
}



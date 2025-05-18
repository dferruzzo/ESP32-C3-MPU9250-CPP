// NVSWrapper.h - ESP32 NVS C++ Wrapper
#pragma once

#include <string>
#include <stdexcept>  // Add this line for std::runtime_error
#include <vector>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_err.h>
#include <esp_log.h>

static constexpr size_t MAX_KEY_LENGTH = 15;

class NVSWrapper {
public:
    /**
     * @brief Constructor - initializes NVS and opens a namespace
     * @param namespace_name The NVS namespace to use (default: "storage")
     * @throw std::runtime_error if initialization fails
     */
    explicit NVSWrapper(const std::string& namespace_name = "storage");
    
    /**
     * @brief Destructor - closes the NVS handle
     */
    ~NVSWrapper();

    // Delete copy constructor and assignment operator
    NVSWrapper(const NVSWrapper&) = delete;
    NVSWrapper& operator=(const NVSWrapper&) = delete;

    /**
     * @brief Write a value to NVS
     * @tparam T Type of the value to store
     * @param key Key to associate with the value
     * @param value Value to store
     * @return true if successful, false otherwise
     */
    template<typename T>
    bool write(const std::string& key, const T& value) {
        esp_err_t err = nvs_set_blob(m_handle, key.c_str(), &value, sizeof(T));
        return handleError(err) && commit();
    }

    /**
     * @brief Read a value from NVS
     * @tparam T Type of the value to read
     * @param key Key associated with the value
     * @param value Reference to store the read value
     * @return true if successful, false otherwise (including if key not found)
     */
    template<typename T>
    bool read(const std::string& key, T& value) {
        size_t required_size = sizeof(T);
        esp_err_t err = nvs_get_blob(m_handle, key.c_str(), &value, &required_size);
        return handleError(err, true);
    }

    /**
     * @brief Write a string to NVS
     * @param key Key to associate with the string
     * @param value String to store
     * @return true if successful, false otherwise
     */
    bool writeString(const std::string& key, const std::string& value);

    /**
     * @brief Read a string from NVS
     * @param key Key associated with the string
     * @param value Reference to store the read string
     * @return true if successful, false otherwise
     */
    bool readString(const std::string& key, std::string& value);

    /**
     * @brief Write a vector to NVS
     * @tparam T Type of elements in the vector
     * @param key Key to associate with the vector
     * @param value Vector to store
     * @return true if successful, false otherwise
     */
    template<typename T>
    bool writeVector(const std::string& key, const std::vector<T>& value) {
        if (value.empty()) {
            return write<size_t>(key + "_size", 0);
        }
        bool success = write<size_t>(key + "_size", value.size());
        if (success) {
            esp_err_t err = nvs_set_blob(m_handle, key.c_str(), value.data(), value.size() * sizeof(T));
            success = handleError(err) && commit();
        }
        return success;
    }

    /**
     * @brief Read a vector from NVS
     * @tparam T Type of elements in the vector
     * @param key Key associated with the vector
     * @param value Reference to store the read vector
     * @return true if successful, false otherwise
     */
    template<typename T>
    bool readVector(const std::string& key, std::vector<T>& value) {
        size_t size = 0;
        if (!read<size_t>(key + "_size", size) || size == 0) {
            value.clear();
            return size == 0;
        }

        value.resize(size);
        size_t required_size = size * sizeof(T);
        esp_err_t err = nvs_get_blob(m_handle, key.c_str(), value.data(), &required_size);
        return handleError(err);
    }

    /**
     * @brief Check if a key exists in NVS
     * @param key Key to check
     * @return true if key exists, false otherwise
     */
    bool keyExists(const std::string& key);

    /**
     * @brief Remove a key from NVS
     * @param key Key to remove
     * @return true if successful, false otherwise
     */
    bool removeKey(const std::string& key);

    /**
     * @brief Erase all keys in the current namespace
     * @return true if successful, false otherwise
     */
    bool eraseAll();

private:
    bool initializeNVS();
    bool commit();
    bool handleError(esp_err_t err, bool ignoreNotFound = false);
    bool validateKey(const std::string& key) {
        if (key.empty() || key.length() > MAX_KEY_LENGTH) {
            ESP_LOGE("NVSWrapper", "Key '%s' is invalid (length %zu). Max length is %zu",
                    key.c_str(), key.length(), MAX_KEY_LENGTH);
            return false;
        }
        return true;
    }
    
    std::string m_namespace;
    nvs_handle_t m_handle;
};

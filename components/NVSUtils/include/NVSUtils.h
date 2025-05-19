#pragma once

#include <eigen3/Eigen/Eigen>
#include "pl_nvs.h"
#include <string>
#include <cstring>

namespace NVSUtils {

inline esp_err_t WriteEigenMatrix(PL::NvsNamespace& nvs, const std::string& key, const Eigen::MatrixXf& mat) {
    // Store rows and cols as metadata
    std::string metaKey = key + "_meta";
    uint32_t dims[2] = {static_cast<uint32_t>(mat.rows()), static_cast<uint32_t>(mat.cols())};
    esp_err_t err = nvs.Write(metaKey, dims, sizeof(dims));
    if (err != ESP_OK) return err;

    // Store matrix data as blob
    return nvs.Write(key, mat.data(), sizeof(float) * mat.size());
}

inline esp_err_t ReadEigenMatrix(PL::NvsNamespace& nvs, const std::string& key, Eigen::MatrixXf& mat) {
    // Read metadata (rows and cols)
    std::string metaKey = key + "_meta";
    uint32_t dims[2] = {0, 0};
    size_t metaSize = sizeof(dims);
    esp_err_t err = nvs.Read(metaKey, dims, sizeof(dims), &metaSize);
    if (err != ESP_OK) return err;

    int rows = static_cast<int>(dims[0]);
    int cols = static_cast<int>(dims[1]);
    if (rows <= 0 || cols <= 0) return ESP_ERR_INVALID_SIZE;

    // Resize matrix
    mat.resize(rows, cols);

    // Read matrix data
    size_t dataSize = sizeof(float) * rows * cols;
    err = nvs.Read(key, mat.data(), dataSize, &dataSize);
    return err;
}

} // namespace NVSUtils
#pragma once

/**
 * @file NVSUtils.h
 * @brief Utility functions for reading/writing Eigen types to NVS (Non-Volatile Storage)
 * 
 * Examples:
 * 
 * // Eigen::Vector3f (fixed size - optimized, no metadata)
 * Eigen::Vector3f gyroBias(0.1f, -0.2f, 0.05f);
 * WriteVector3f(nvs, "gyro_bias", gyroBias);
 * 
 * Eigen::Vector3f readBias;
 * ReadVector3f(nvs, "gyro_bias", readBias);
 * 
 * // Eigen::VectorXf (dynamic size - stores size as metadata)
 * Eigen::VectorXf myVector(10);
 * myVector.setRandom();
 * WriteEigenVector(nvs, "my_data", myVector);
 * 
 * Eigen::VectorXf loadedVector;
 * ReadEigenVector(nvs, "my_data", loadedVector);
 * 
 * // Eigen::MatrixXf
 * Eigen::MatrixXf calibMatrix(4, 3);
 * calibMatrix.setIdentity();
 * WriteEigenMatrix(nvs, "calib_matrix", calibMatrix);
 * 
 * Eigen::MatrixXf loadedMatrix;
 * ReadEigenMatrix(nvs, "calib_matrix", loadedMatrix);
 */

#include <eigen3/Eigen/Eigen>
#include "pl_nvs.h"
#include <string>
#include <cstring>

namespace NVSUtils {

// a função to read a float value from NVS
inline esp_err_t ReadFloat(PL::NvsNamespace& nvs, const std::string& key, float& value) {
    size_t dataSize = sizeof(value);
    return nvs.Read(key, &value, dataSize, &dataSize);
}

// a function to write a float value to NVS
inline esp_err_t WriteFloat(PL::NvsNamespace& nvs, const std::string& key, float value) {
    return nvs.Write(key, &value, sizeof(value));
}

// a function to write a 3-element float vector to NVS
inline esp_err_t Write3fVector(PL::NvsNamespace& nvs, const std::string& key, const float vec[3]) {
    return nvs.Write(key, vec, sizeof(float) * 3);
}

// a function to read a 3-element float vector from NVS
inline esp_err_t Read3fVector(PL::NvsNamespace& nvs, const std::string& key, float vec[3]) {
    size_t dataSize = sizeof(float) * 3;
    return nvs.Read(key, vec, dataSize, &dataSize);
}

inline esp_err_t WriteBool(PL::NvsNamespace& nvs, const std::string& key, bool value) {
    uint8_t val = value ? 1 : 0;
    return nvs.Write(key, &val, sizeof(val));
}

inline esp_err_t ReadBool(PL::NvsNamespace& nvs, const std::string& key, bool& value) {
    uint8_t val = 0;
    size_t size = sizeof(val);
    esp_err_t err = nvs.Read(key, &val, sizeof(val), &size);
    if (err == ESP_OK) {
        value = (val != 0);
    }
    return err;
}

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

// Write Eigen::Vector (VectorXf, Vector3f, etc.) to NVS
// Usage: WriteEigenVector(nvs, "my_vector", myVectorXf);
//        WriteEigenVector(nvs, "gyro_bias", gyroVector3f);
template<typename Derived>
inline esp_err_t WriteEigenVector(PL::NvsNamespace& nvs, const std::string& key, const Eigen::MatrixBase<Derived>& vec) {
    // Store size as metadata
    std::string metaKey = key + "_size";
    uint32_t size = static_cast<uint32_t>(vec.size());
    esp_err_t err = nvs.Write(metaKey, &size, sizeof(size));
    if (err != ESP_OK) return err;

    // Store vector data as blob
    return nvs.Write(key, vec.derived().data(), sizeof(typename Derived::Scalar) * vec.size());
}

// Read Eigen::Vector (VectorXf, Vector3f, etc.) from NVS
// Usage: Eigen::VectorXf vec;
//        ReadEigenVector(nvs, "my_vector", vec);
template<typename VectorType>
inline esp_err_t ReadEigenVector(PL::NvsNamespace& nvs, const std::string& key, VectorType& vec) {
    // Read metadata (size)
    std::string metaKey = key + "_size";
    uint32_t size = 0;
    size_t metaSize = sizeof(size);
    esp_err_t err = nvs.Read(metaKey, &size, sizeof(size), &metaSize);
    if (err != ESP_OK) return err;

    if (size <= 0) return ESP_ERR_INVALID_SIZE;

    // Resize vector (only for dynamic vectors like VectorXf)
    if (vec.size() != size) {
        vec.resize(size);
    }

    // Read vector data
    size_t dataSize = sizeof(typename VectorType::Scalar) * size;
    err = nvs.Read(key, vec.data(), dataSize, &dataSize);
    return err;
}

// Write Eigen::Vector3f (fixed size) to NVS - optimized version (no metadata)
// Usage: WriteVector3f(nvs, "gyro_bias", gyroBias);
inline esp_err_t WriteVector3f(PL::NvsNamespace& nvs, const std::string& key, const Eigen::Vector3f& vec) {
    return nvs.Write(key, vec.data(), sizeof(float) * 3);
}

// Read Eigen::Vector3f (fixed size) from NVS - optimized version (no metadata)
// Usage: Eigen::Vector3f bias;
//        ReadVector3f(nvs, "gyro_bias", bias);
inline esp_err_t ReadVector3f(PL::NvsNamespace& nvs, const std::string& key, Eigen::Vector3f& vec) {
    size_t dataSize = sizeof(float) * 3;
    return nvs.Read(key, vec.data(), dataSize, &dataSize);
}

} // namespace NVSUtils
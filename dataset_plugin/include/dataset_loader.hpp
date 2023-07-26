# pragma once

#include <cassert>
#include <cstddef>
#include <chrono>
#include <filestystem>
#include <map>
#include <string>
#include <unordered_map>

#include <opencv2/core/mat.hpp>
#include <eigen3/Eigen/Dense>

#include "config.hpp"
#include "common/data_format.hpp"

enum class ImageType {
    RGB,
    Depth,
    Grayscale
};

// final output format is fixed, and is defined by the structs in `common/data_format.hpp`, but
// intermediate data formats can be defined by me for my convenience.
struct ImageData {
    ImageData() = default;

    ImageData(const std::string& path, int channel, ImageType type)
        : m_path(path), m_channel(channel), m_type(type) { }

    cv::Mat loadImage() {
        if (m_type == ImageType::Grayscale) {
            return loadGrayscale();
        } else if (m_type == ImageType::RGB) {
            return loadRGB();
        } else {
            // the only remaining option is that it's a depth image
            loadDepth();
        }
    }

    std::size_t getChannel() {
        return m_channel;
    }

private:
    cv::Mat loadGrayscale() {
        m_mat = cv::imread(m_path, cv::IMREAD_GRAYSCALE);

        assert(!m_mat.empty());
        
        return m_mat;
    }

    cv::Mat loadRGB() {
        m_mat = cv::imread(m_path, cv::IMREAD_COLOR);

        assert(!m_mat.empty());
        
        return m_mat;
    }

    cv::Mat loadDepth() {
        m_mat = cv::imread(m_path, cv::IMREAD_UNCHANGED);

        assert(!m_mat.empty());
        
        return m_mat;
    }

    std::string m_path;
    cv::Mat m_mat;
    std::size_t m_channel;
    
    // tells us what load function to use to load the image
    ImageType m_type;
};


struct IMUData {
    Eigen::Vector3d angular_v;
    Eigen::Vector3d linear_a;
    std::size_t channel;
};


struct PoseData {
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    std::size_t channel;
};

struct GroundTruthData {
    std::unordered_map<std::string, Eigen::VectorXd> data;
};

class DatasetLoader {
    // might make more sense to rename the class into something else
public:
    DatasetLoader(const DatasetLoader&) = delete;
    DatasetLoader& operator=(const DatasetLoader&) = delete;
    
    static DatasetLoader* getInstance() {
        if ( !instance ){
        instance = new DatasetLoader();
      }
      return instance;
    }

    std::multimap<std::chrono::nanoseconds, IMUData> getIMUData() {
        return m_IMUData;
    }

    std::multimap<std::chrono::nanoseconds, ImageData> getImageData() {
        return m_imageData;
    }
    
    std::multimap<std::chrono::nanoseconds, PoseData> getPoseData() {
        return m_poseData;
    }
    
    std::multimap<std::chrono::nanoseconds, GroundTruthData> getGroundTruthData() {
        return m_groundTruthData;
    }

private:
    DatasetLoader() {
        ConfigParser parser;

        parser.initFromConfig(m_config);

        loadIMUData();
        loadImageData();
        loadPoseData();
        loadGroundTruthData();
    }

    inline static DatasetLoader* instance{nullptr};

    // these functions shouldn't be called from outside the class. They are called
    // when an object of this class is initialized.
    void loadIMUData();

    void loadImageData();

    void loadPoseData();

    void loadGroundTruthData();
    
    // helper functions
    bool isImageFile(const std::filesystem::path&);

    std::chrono::nanoseconds convertToTimestamp(const TimestampUnit&, unsigned long long);

    Config m_config;
    
    std::multimap<std::chrono::nanoseconds, IMUData> m_IMUData;
    std::multimap<std::chrono::nanoseconds, ImageData> m_imageData;
    std::multimap<std::chrono::nanoseconds, PoseData> m_poseData;
    std::multimap<std::chrono::nanoseconds, GroundTruthData> m_groundTruthData;
};
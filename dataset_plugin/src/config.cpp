#include <string>

#include "config.hpp"

void ConfigParser::initIMUConfig() {


    // parsing format-related info TODO: Finish this
    const char* format_env_var = std::getenv("ILLIXR_DATASET_POSE_PATH");
    if (!datatype_env_var) {
        std::cerr << "Error: Please define `ILLIXR_DATASET_POSE_PATH`." << std::endl;
        ILLIXR::abort();
    }
}

void ConfigParser::initImageConfig() {
    // parsing timestamp unit-related info.
    const char* timestamp_units_env_var = std::getenv("ILLIXR_DATASET_IMAGE_TIMESTAMP_UNITS");
    if (!timestamp_env_var) {
        std::cerr << "Error: Please define `ILLIXR_DATASET_IMAGE_TIMESTAMP_UNITS`." << std::endl;
        ILLIXR::abort();
    }

    std::string timestamp_units{timestamp_units_env_var};

    if (timestamp_units == "seconds") {
        config.image_config.timestamp_unit = TimestampUnit::second;
    } else if (timestamp_units == "milliseconds") {
        config.image_config.timestamp_unit = TimestampUnit::millisecond;
    } else if (timestamp_units == "microseconds") {
        config.image_config.timestamp_unit = TimestampUnit::microsecond;
    } else {
        // nanoseconds
        config.image_config.timestamp_unit = TimestampUnit::nanosecond;
    }

    // parsing image path-related info.

    // rgb path(s)
    const char* rgb_path_env_var = std::getenv("ILLIXR_DATASET_IMAGE_RGB_PATH");
    if (!rgb_path_env_var) {
        std::cerr << "Error: Please define `ILLIXR_DATASET_IMAGE_RGB_PATH`." << std::endl;
        ILLIXR::abort();
    }

    config.image_config.rgb_path_list = convertPathStringToPathList(rgb_path_env_var);

    // depth path(s)
    const char* depth_path_env_var = std::getenv("ILLIXR_DATASET_IMAGE_DEPTH_PATH");
    if (!depth_path_env_var) {
        std::cerr << "Error: Please define `ILLIXR_DATASET_IMAGE_DEPTH_PATH`." << std::endl;
        ILLIXR::abort();
    }

    config.image_config.depth_path_list = convertPathStringToPathList(depth_path_env_var);

    // grayscale path(s)
    const char* grayscale_path_env_var = std::getenv("ILLIXR_DATASET_IMAGE_GRAYSCALE_PATH");
    if (!grayscale_path_env_var) {
        std::cerr << "Error: Please define `ILLIXR_DATASET_IMAGE_GRAYSCALE_PATH`." << std::endl;
        ILLIXR::abort();
    }

    config.image_config.grayscale_path_list = convertPathStringToPathList(grayscale_path_env_var);
}

void ConfigParser::initPoseConfig() {
    // parsing datatype-related info.
    const char* datatype_env_var = std::getenv("ILLIXR_DATASET_POSE_DATATYPE");
    if (!datatype_env_var) {
        std::cerr << "Error: Please define `ILLIXR_DATASET_POSE_DATATYPE`." << std::endl;
        ILLIXR::abort();
    }

    std::string datatype{datatype_env_var};

    if (datatype == "double") {
        config.pose_config.use_double = true;
    }
    // no need to do anything otherwise, since the other option is to use `float`, which is
    // the default choice.

    // parsing timestamp unit-related info.
    const char* timestamp_units_env_var = std::getenv("ILLIXR_DATASET_POSE_TIMESTAMP_UNITS");
    if (!timestamp_env_var) {
        std::cerr << "Error: Please define `ILLIXR_DATASET_POSE_TIMESTAMP_UNITS`." << std::endl;
        ILLIXR::abort();
    }

    std::string timestamp_units{timestamp_units_env_var};

    if (timestamp_units == "seconds") {
        config.pose_config.timestamp_unit = TimestampUnit::second;
    } else if (timestamp_units == "milliseconds") {
        config.pose_config.timestamp_unit = TimestampUnit::millisecond;
    } else if (timestamp_units == "microseconds") {
        config.pose_config.timestamp_unit = TimestampUnit::microsecond;
    } else {
        // nanoseconds
        config.pose_config.timestamp_unit = TimestampUnit::nanosecond;
    }

    // parsing path-related info.
    const char* path_env_var = std::getenv("ILLIXR_DATASET_POSE_PATH");
    if (!datatype_env_var) {
        std::cerr << "Error: Please define `ILLIXR_DATASET_POSE_PATH`." << std::endl;
        ILLIXR::abort();
    }

    config.pose_config.pose_path_list = convertPathStringToPathList(path_env_var);
}

void ConfigParser::initGroundTruthConfig() {

}

void ConfigParser::initFromConfig() {
    // delimiter
    const char* delimiter = std::getenv("ILLIXR_DATASET_DELIMITER");
    if (!delimiter) {
        std::cerr << "Error: Please define `ILLIXR_DATASET_DELIMITER`." << std::endl;
        ILLIXR::abort();
    }

    config.delimiter = delimiter[0];

    // root path
    const char* root_path = std::getenv("ILLIXR_DATASET_ROOT_PATH");
    if (!root_path) {
        std::cerr << "Error: Please define `ILLIXR_DATASET_ROOT_PATH`." << std::endl;
        ILLIXR::abort();
    }

    config.root_path = root_path;

    initIMUConfig();

    initImageConfig();

    initPoseConfig();

    initGroundTruthConfig();
}
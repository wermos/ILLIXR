#include "common/csv_iterator.hpp"

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <map>
#include <optional>
#include <string>

typedef unsigned long long ullong;

struct state_data {
    ullong timestamp;
    Eigen::Vector3d center;
    Eigen::Vector4d orientation;
};

struct rgb_image_data {
    ullong timestamp;
    std::string path;
};

struct depth_image_data {
    ullong timestamp;
    std::string path;
};

struct ground_truth_data {
    state_data state;
    rgb_image_data rgb_image;
    depth_image_data depth_image;
};

static std::map<ullong, ground_truth_data> load_data() {
    // fix this path business
    // const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
    // if (!illixr_data_c_str) {
    //     std::cerr << "Please define ILLIXR_DATA" << std::endl;
    //     ILLIXR::abort();
    // }
    // std::string illixr_data = std::string{illixr_data_c_str};

    std::map<ullong, ground_truth_data> data;

    // const std::string data_subpath = "/TUM-RGBD/final.csv";
    // std::ifstream     data_file{illixr_data + data_subpath};
    
    std::ifstream     data_file{".cache/paths/TUM-RGBD/final.csv"};

    if (!data_file.good()) {
        // std::cerr << "${ILLIXR_DATA}" << data_subpath << " (" << illixr_data << data_subpath << ") is not a good path"
        //           << std::endl;
        std::cerr << "${ILLIXR_DATA} is not a good path" << std::endl;
        ILLIXR::abort();
    }
    for (CSVIterator row{data_file, 1}; row != CSVIterator{}; ++row) {
        // position + orientation stuff
        ullong timestamp1 = std::stoull(row[0]);
        Eigen::Vector3d center{std::stod(row[1]), std::stod(row[2]), std::stod(row[3])};
        Eigen::Vector3d orientation{std::stod(row[4]), std::stod(row[5]), std::stod(row[6]), std::stod(row[7])};

        // depth image stuff
        ullong timestamp2 = std::stoull(row[8]);
        std::string depth_image_path = row[9];

        // rgb image stuff
        ullong timestamp3 = std::stoull(row[10]);
        std::string rgb_image_path = row[11];


        data[timestamp1].state = {timestamp1, center, orientation};
        data[timestamp1].depth_image = {timestamp2, depth_image_path};
        data[timestamp1].rgb_image = {timestamp3, rgb_image_path};
    }

    for (int i = 0; i < 10; ++i) {
        std::cout << data[i].state.timestamp << ' ' << data[i].state.center << ' ' << data[i].state.orientation << '\n';

        std::cout << data[i].depth_image.timestamp << ' ' << data[i].depth_image.path << '\n';

        std::cout << data[i].rgb_image.timestamp << ' ' << data[i].rgb_image.path << '\n';
    }

    return data;
}

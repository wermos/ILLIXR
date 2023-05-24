#include <map>
#include <fstream>
#include <string>
#include <optional>
#include <iostream>
#include <cassert>


#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <eigen3/Eigen/Dense>

#include "common/csv_iterator.hpp"
#include "common/data_format.hpp"

using ullong = unsigned long long;

using namespace ILLIXR;

class lazy_load_image {
public:
    lazy_load_image()
        : _m_path("")
    {}

    lazy_load_image(const std::string& path)
        : _m_path(path)
    {}

    std::unique_ptr<cv::Mat> unmodified_load() const {
        auto img = std::unique_ptr<cv::Mat>{new cv::Mat{cv::imread(_m_path, cv::IMREAD_UNCHANGED)}};

        assert(!img->empty());

        return img;
    }

    std::unique_ptr<cv::Mat> modified_load() const {
        // read the cv::Mat in RGB format and copy and covert it to 4 channel format used in ILLIXR
        cv::Mat original_mat = cv::imread(_m_path, cv::IMREAD_UNCHANGED);
        cv::Mat* converted_mat = new cv::Mat();

        cv::cvtColor(original_mat, *converted_mat, cv::COLOR_RGB2RGBA,0);
        auto img = std::unique_ptr<cv::Mat>{converted_mat};

        assert(!img->empty());

        return img;
    }

    const std::string get_path() const {
        return _m_path;
    }
private:
    std::string _m_path;
};

struct sensor_types {
    pose_type            pose;
    lazy_load_image depth_cam;
    lazy_load_image color_cam;
};


static std::map<ullong, sensor_types> load_data() {
    const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");

    if (!illixr_data_c_str) {
        std::cerr << "Please define ILLIXR_DATA" << std::endl;
        ILLIXR::abort();
    }

    std::string illixr_data = std::string{illixr_data_c_str};
    const std::string dataset_subpath = "/final.csv";

    std::ifstream dataset_file{illixr_data + dataset_subpath};

    if (!dataset_file.good()) {
        std::cerr << "${ILLIXR_DATA}" << dataset_subpath << " (" << illixr_data << dataset_subpath << ") is not a good path" << std::endl;
        ILLIXR::abort();
    }

    std::clog << "loading pose file: " << illixr_data << dataset_subpath << '\n';

    std::string timestamp;

    std::map<ullong, sensor_types> data;

    //pyh: VCU & TUM does not have row as legends for units => not skipping the first line 
    for (CSVIterator row{dataset_file, 0}; row != CSVIterator{}; ++row) {
        // first, we convert the timestamp to a string
        timestamp = row[0].c_str();

        auto decimal_start = timestamp.find("."); // find where the decimal point is in the string.

        if (decimal_start != std::string::npos) {
            timestamp.erase(decimal_start, 1);
        }

        ullong t = std::stoull(timestamp);

        // locate depth image location
        lazy_load_image depth_cam = {illixr_data +  "/" + row[9]};
        data[t].depth_cam = depth_cam;
        
        // locate color image location
        lazy_load_image color_cam = {illixr_data +  "/" + row[11]};
        data[t].color_cam = color_cam;
        
        Eigen::Vector3f position{std::stof(row[1]), std::stof(row[2]), std::stof(row[3])};
        Eigen::Quaternionf orientation{std::stof(row[4]), std::stof(row[5]), std::stof(row[6]), 
                                       std::stof(row[7])};
        
        data[t].pose = {time_point{}, position, orientation};
    }

    std::clog << "Finished data loading size: " << data.size() << std::endl;

    return data;
}

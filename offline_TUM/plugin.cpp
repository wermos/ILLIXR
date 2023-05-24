#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/threadloop.hpp"
#include "common/global_module_defs.hpp"

#include "data_loading.hpp"

#include <cassert>
#include <thread>
#include <chrono>

#include <eigen3/Eigen/Dense>

using namespace ILLIXR;

// const record_header imu_cam_record {
// 	"imu_cam",
// 		{
// 			{"iteration_no", typeid(std::size_t)},
// 			{"has_depth", typeid(bool)},
// 			{"has_color", typeid(bool)},
// 		},
// };

class offline_TUM : public ILLIXR::threadloop {
public:
	offline_TUM(std::string name_, phonebook* pb_)
		: threadloop{name_, pb_}
	, _m_sb{pb->lookup_impl<switchboard>()}
	, _m_sensor_data{load_data()}
	, _m_sensor_data_it{_m_sensor_data.cbegin()}
	, _m_clock{pb->lookup_impl<RelativeClock>()} 
	, _m_rgb_depth{_m_sb->get_writer<rgb_depth_type>("rgb_depth")} 
	, dataset_first_time{_m_sensor_data_it->first}
	, dataset_prev{_m_sensor_data_it->first}
	, imu_cam_log{record_logger_}
	, camera_cvtfmt_log{record_logger_}
	{}

protected:
	virtual skip_option _p_should_skip() override {
		if (_m_sensor_data_it != _m_sensor_data.end()) {
			dataset_now = _m_sensor_data_it->first;
			std::this_thread::sleep_for(
					std::chrono::nanoseconds{dataset_now - dataset_first_time}
					+ real_first_time
					- std::chrono::steady_clock::now()
					);
			// For TUM, since we have no actual IMU reading, we use depth image to filter
			// if (_m_sensor_data_it->second.depth_cam != lazy_load_image{}) {
			// 	return skip_option::run;
			// } else {
			// 	++_m_sensor_data_it;
			// 	return skip_option::skip_and_yield;
			// }
			return skip_option::run;
		} else {
			std::cout << "Stopped due to end of frame\n";
			return skip_option::stop;
		}
	}

	virtual void _p_one_iteration() override {
		RAC_ERRNO_MSG("offline_TUM at start of _p_one_iteration");
		assert(_m_sensor_data_it != _m_sensor_data.end());
#ifndef NDEBUG
		std::chrono::time_point<std::chrono::nanoseconds> tp_dataset_now{std::chrono::nanoseconds{dataset_now}};
		//std::cerr << " IMU time: " << tp_dataset_now.time_since_epoch().count() << std::endl;
#endif /// NDEBUG
		// pyh: some debugging codes (best to follow sam's format above)
		//		time_type real_now = real_first_time + std::chrono::nanoseconds{dataset_now - dataset_first_time};
		//		std::cout<< "dataset now: " << dataset_now << " count: " << count << std::endl;
		
		
		// count++;
		const sensor_types& sensor_datum = _m_sensor_data_it->second;
		// const ullong timestamp = _m_sensor_data_it->first;
		++_m_sensor_data_it;

		// imu_cam_log.log(record{imu_cam_record, {
		// 		{iteration_no},
		// 		{bool(sensor_datum.cam0)},
		// 		{bool(sensor_datum.cam1)},
		// 		}});

		cv::Mat depth = *(sensor_datum.depth_cam.unmodified_load().release());

		RAC_ERRNO_MSG("offline_TUM after depth");

		cv::Mat rgb = *(sensor_datum.color_cam.unmodified_load().release());

		RAC_ERRNO_MSG("offline_TUM after rgb");

		//pyh: send the data structure to the scoreboard    
		_m_rgb_depth.put(_m_rgb_depth.allocate<rgb_depth_type>(
					rgb_depth_type{
					_m_clock->now(),
					rgb,
					depth			
					}));

		// RAC_ERRNO_MSG("offline_TUM at bottom of iteration");
		dataset_prev = dataset_now;
	}	

public:
/** fix this time bullshit */
	virtual void _p_thread_setup() override {
		// this is not done in the constructor, because I want it to
		// be done at thread-launch time, not load-time.

		// auto now = std::chrono::steady_clock::now();
		// auto real_first_time = std::chrono::time_point_cast<std::chrono::seconds>(now);

		real_first_time = std::chrono::steady_clock::now();

		//pyh: some debug code
		//std::cout << "real first time: " << real_first_time.time_since_epoch().count() << '\n';
	}

private:
	// // pyh: data structure for holding incomign frame
	// switchboard::writer<rgb_depth_type> _m_rgb_depth;

	// // Timestamp of the first IMU value from the dataset
	// ullong dataset_first_time;
	// // UNIX timestamp when this component is initialized
	// time_point real_first_time;
	// // Current IMU timestamp
	// ullong dataset_now;
	// ullong dataset_prev = 0;
	// //pyh: track # of frames triggered 
	// unsigned int count = 0;

	const std::shared_ptr<switchboard>             _m_sb;
	const std::map<ullong, sensor_types>           _m_sensor_data;
	std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
	std::shared_ptr<const RelativeClock>           _m_clock;
	switchboard::writer<rgb_depth_type>            _m_rgb_depth;


	// Timestamp of the first position and image value from the dataset
	ullong dataset_first_time;

	// UNIX timestamp when this component is initialized
	std::chrono::steady_clock::time_point real_first_time;

	// Current data timestamp
	ullong dataset_now;
	ullong dataset_prev = 0;

	record_coalescer imu_cam_log;
    record_coalescer camera_cvtfmt_log;
};

PLUGIN_MAIN(offline_TUM)

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>

#include <cstring>
#include <cmath>

using sensor_msgs::LaserScan;
using sensor_msgs::LaserScanPtr;

namespace selfie_obstacle_detection
{

class RemoveBackgroundNodelet : public nodelet::Nodelet
{
	float low_threshold_;
	float high_threshold_;
	float min_size_;
	float max_size_;

	ros::Subscriber sub_;
	ros::Publisher pub_;

	virtual void onInit();
	void processScan(const LaserScanPtr& in_scan);
}; // class RemoveBackgroundNodelet

void RemoveBackgroundNodelet::onInit()
{
	ros::NodeHandle& nh         = getNodeHandle();
	ros::NodeHandle& private_nh = getPrivateNodeHandle();

	// Read parameters
	private_nh.param("low_threshold",  low_threshold_, -0.25f);
	private_nh.param("high_threshold", high_threshold_, 0.25f);
	private_nh.param("min_size",       min_size_,       0.075f);
	private_nh.param("max_size",       max_size_,       0.5f);

	// Subscribe to incoming scans and republish after filtration
	sub_ = nh.subscribe           ("scan", 10, &RemoveBackgroundNodelet::processScan, this);
	pub_ = nh.advertise<LaserScan>("scan_filtered", 10);
}

inline void appendZeros(std::vector<float>& v, int n)
{
	for (int i = 0; i < n; i++) v.push_back(0);
}

inline LaserScanPtr constructNewMessage(const LaserScanPtr in_scan)
{
	LaserScanPtr out_scan(new LaserScan);

	out_scan->header          = in_scan->header;

	out_scan->angle_min       = in_scan->angle_min;
	out_scan->angle_max       = in_scan->angle_max;
	out_scan->angle_increment = in_scan->angle_increment;

	out_scan->time_increment  = in_scan->time_increment;
	out_scan->scan_time       = in_scan->scan_time;

	out_scan->range_min       = in_scan->range_min;
	out_scan->range_max       = in_scan->range_max;

	out_scan->intensities     = in_scan->intensities;

	return out_scan;
}

void RemoveBackgroundNodelet::processScan(const LaserScanPtr& in_scan)
{
	NODELET_DEBUG("NEW SCAN");
	
	int n_readings = in_scan->ranges.size();

	// We aren't supposed to modify the incoming message,
	// so we need to create a new one to be republished
	LaserScanPtr out_scan = constructNewMessage(in_scan);

	bool interesting_segment = false;
	int start = 0;
	for (int i = 1; i < n_readings - 1; i++)
	{
		float range_diff = in_scan->ranges[i + 1] - in_scan->ranges[i - 1];

		if (range_diff < low_threshold_)
		{
			NODELET_DEBUG("Low diff: %f m", i, range_diff);

			// Fill with zeros for [start, i)
			appendZeros(out_scan->ranges, i - start);

			interesting_segment = true;
			start = i;
			continue;
		}

		if (range_diff > high_threshold_ && interesting_segment)
		{
			NODELET_DEBUG("High diff: %f m", i, range_diff);

			float start_angle = in_scan->angle_min + start * in_scan->angle_increment;
			float stop_angle  = in_scan->angle_min + i     * in_scan->angle_increment;

			float start_x = in_scan->ranges[start] * std::cos(start_angle);
			float start_y = in_scan->ranges[start] * std::sin(start_angle);

			float stop_x = in_scan->ranges[i] * std::cos(stop_angle);
			float stop_y = in_scan->ranges[i] * std::sin(stop_angle);

			float segment_size = std::sqrt(std::pow(stop_x - start_x, 2)
			                             + std::pow(stop_y - start_y, 2));

			NODELET_DEBUG("Start: %f rad, %f m", start_angle, in_scan->ranges[start]);
			NODELET_DEBUG("Stop:  %f rad, %f m", stop_angle,  in_scan->ranges[i]);
			NODELET_DEBUG("Size: %f m", segment_size);

			if (min_size_ <= segment_size && segment_size <= max_size_)
			{
				// Preserve current scan segment as it is a good obstacle candidate
				// Copy range values from source scan for [start, i]
				for (int j = start; j < i + 1; j++) out_scan->ranges.push_back(in_scan->ranges[j]);
			}
			else
			{
				// Fill with zeros for [start, i]
				appendZeros(out_scan->ranges, i + 1 - start);
			}

			interesting_segment = false;
			start = i + 1;
		}
	}

	// Pad with zeros
	appendZeros(out_scan->ranges, n_readings - start);

	// Republish filtered scan
	pub_.publish(out_scan);
}

} // namespace selfie_obstacle_detection

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(selfie_obstacle_detection::RemoveBackgroundNodelet, nodelet::Nodelet)

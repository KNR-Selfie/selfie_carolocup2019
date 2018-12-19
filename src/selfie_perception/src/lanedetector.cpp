#include <selfie_perception/lanedetector.h>

#define TOPVIEW_ROWS 480
#define TOPVIEW_COLS 640

#define TOPVIEW_MIN_X  0.0
#define TOPVIEW_MAX_X  1.3
#define TOPVIEW_MIN_Y -1.5
#define TOPVIEW_MAX_Y  1.5

static int Acc_slider = 1;
static int Acc_value = 1;
static int Acc_filt = 5;
static int Acc_filt_slider = 40;

poly left_line_poly_;
poly center_line_poly_;
poly right_line_poly_;

LaneDetector::LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
	nh_(nh),
	pnh_(pnh),
	it_(nh),
	binary_treshold_(175),
	visualize_(true),
	max_mid_line_gap_(150),
	max_mid_line_distance_(35),
	init_imageCallback_(true),

	min_length_search_line_(30),
	min_length_lane_(67),
	max_delta_y_lane_(30),
	nominal_center_line_Y_(10),

	left_line_index_(-1),
	right_line_index_(-1),
	center_line_index_(-1),

	kernel_v_(),
	current_frame_(),
	gray_frame_(),
	binary_frame_(),
	dynamic_mask_(),
	masked_frame_(),
	crossing_ROI_(),
	crossing_frame_(),
	canny_frame_(),
	visualization_frame_(),
	homography_frame_()
{
	lanes_pub_ =  nh_.advertise<selfie_msgs::RoadMarkings>("road_markings", 10);
	if(visualize_)
	{
		points_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("new_coordinates", 10);
		aprox_visualization_pub_ = nh_.advertise<visualization_msgs::Marker>("aprox", 10);
	}

}

LaneDetector::~LaneDetector()
{
	cv::destroyAllWindows();
}

bool LaneDetector::init()
{
	lanes_vector_.clear();
	lanes_vector_last_frame.clear();
	aprox_lines_frame_coordinate_.clear();
	std::vector<cv::Point> empty;
	empty.clear();
	lanes_vector_last_frame.push_back(empty);
	lanes_vector_last_frame.push_back(empty);
	lanes_vector_last_frame.push_back(empty);
	aprox_lines_frame_coordinate_.push_back(empty);
	aprox_lines_frame_coordinate_.push_back(empty);
	aprox_lines_frame_coordinate_.push_back(empty);

	kernel_v_ = cv::Mat(1, 3, CV_32F);
	kernel_v_.at<float>(0, 0) = -1;
	kernel_v_.at<float>(0, 1) = 0;
	kernel_v_.at<float>(0, 2) = 1;

	pnh_.getParam("config_file", config_file_);
	pnh_.getParam("binary_treshold", binary_treshold_);
	pnh_.getParam("visualize", visualize_);
	pnh_.getParam("max_mid_line_gap", max_mid_line_gap_);

	image_sub_ = it_.subscribe("/image_rect", 1, &LaneDetector::imageCallback, this);

	cv::FileStorage fs(config_file_, cv::FileStorage::READ);
	fs["world2cam"] >> world2cam_;
	fs.release();

	computeTopView();

	printInfoParams();
	return true;
}

void LaneDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		current_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	homography(current_frame_, homography_frame_);
	cv::threshold(homography_frame_, binary_frame_, binary_treshold_, 255, cv::THRESH_BINARY);

	if(!init_imageCallback_)
	{
		dynamicMask(binary_frame_, masked_frame_, aprox_lines_frame_coordinate_);
		crossingLane(binary_frame_, crossing_frame_, aprox_lines_frame_coordinate_);

		testCrossing = cv::Mat::zeros(homography_frame_.size(),homography_frame_.type());
		cv::bitwise_or(homography_frame_,homography_frame_,testCrossing, crossing_ROI_);
		cv::bitwise_not(crossing_ROI_,crossing_ROI_);

		cv::bitwise_and(masked_frame_,crossing_ROI_,masked_frame_);
	}
	else
		masked_frame_ = binary_frame_.clone();

	cv::medianBlur(masked_frame_, masked_frame_, 5);
	cv::filter2D(masked_frame_, canny_frame_, -1, kernel_v_, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);


	detectLines(canny_frame_, lanes_vector_);
	filterSmallLines();

	
	

	if(!lanes_vector_.empty())
	{
		mergeMiddleLane();
		convertCoordinates();
		if(init_imageCallback_)
		{
			initRecognizeLines();
			if(!visualize_)
				init_imageCallback_ = false;
		}
		else
		{
			recognizeLines();
			filterPoints();
		}
		linesApproximation(lanes_vector_);
		calcValuesForMasks();
	}

	publishMarkings();

	if (visualize_)
	{
		visualization_frame_.rows = homography_frame_.rows;
		visualization_frame_.cols = homography_frame_.cols;

		drawPoints(visualization_frame_);

		openCVVisualization();
		aproxVisualization();
		pointsRVIZVisualization();
	}
}

void LaneDetector::computeTopView()
{
	// Choose top-view image size
	topview_size_ = cv::Size(TOPVIEW_COLS, TOPVIEW_ROWS);

	// Choose corner points (in real-world coordinates)
	std::vector<cv::Point2f> coordinates;
	coordinates.emplace_back(TOPVIEW_MIN_X, TOPVIEW_MIN_Y);
	coordinates.emplace_back(TOPVIEW_MIN_X, TOPVIEW_MAX_Y);
	coordinates.emplace_back(TOPVIEW_MAX_X, TOPVIEW_MIN_Y);
	coordinates.emplace_back(TOPVIEW_MAX_X, TOPVIEW_MAX_Y);

	std::vector<cv::Point2f> pixels;
	pixels.emplace_back(topview_size_.width, topview_size_.height);
	pixels.emplace_back(0, topview_size_.height);
	pixels.emplace_back(topview_size_.width, 0);
	pixels.emplace_back(0, 0);

	topview2world_ = cv::findHomography(pixels, coordinates);

	topview2cam_ = world2cam_ * topview2world_;
}

void LaneDetector::detectLines(cv::Mat &input_frame, std::vector<std::vector<cv::Point> > &output_lanes)
{
	output_lanes.clear();
	cv::findContours(input_frame, output_lanes, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	for (std::vector<std::vector<cv::Point> >::iterator filt = output_lanes.begin(); filt != output_lanes.end();)
	{
		if (filt->size() < Acc_filt)
			filt = output_lanes.erase(filt);
		else
			++filt;
	}
	for (int i = 0; i < output_lanes.size(); i++)
	{
		cv::approxPolyDP(cv::Mat(output_lanes[i]), output_lanes[i], Acc_value, false);
	}
}

void LaneDetector::drawPoints(cv::Mat &frame)
{
	frame = cv::Mat::zeros(frame.size(), CV_8UC3);
	if(!aprox_lines_frame_coordinate_[2].empty())
		for (int i = 0; i < aprox_lines_frame_coordinate_[2].size(); i++)
		{
			cv::circle(frame, aprox_lines_frame_coordinate_[2][i], 3, cv::Scalar(255, 0, 0), CV_FILLED, cv::LINE_AA);
		}
	if(!aprox_lines_frame_coordinate_[0].empty())
		for (int i = 0; i < aprox_lines_frame_coordinate_[0].size(); i++)
		{
			cv::circle(frame, aprox_lines_frame_coordinate_[0][i], 3, cv::Scalar(0, 0, 255), CV_FILLED, cv::LINE_AA);
		}
	if(!aprox_lines_frame_coordinate_[1].empty())
		for (int i = 0; i < aprox_lines_frame_coordinate_[1].size(); i++)
		{
			cv::circle(frame, aprox_lines_frame_coordinate_[1][i], 3, cv::Scalar(0, 255, 0), CV_FILLED, cv::LINE_AA);
		}
}

void LaneDetector::homography(cv::Mat input_frame, cv::Mat &homography_frame)
{
	cv::warpPerspective(input_frame, homography_frame, topview2cam_, topview_size_, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
}

void LaneDetector::openCVVisualization()
{
	
	cv::namedWindow("Binarization", cv::WINDOW_NORMAL);
	cv::imshow("Binarization", binary_frame_);

	cv::namedWindow("masked", cv::WINDOW_NORMAL);
	cv::imshow("masked", masked_frame_);

	cv::namedWindow("Output", cv::WINDOW_NORMAL);
	cv::imshow("Output", visualization_frame_);

	cv::namedWindow("Homography", cv::WINDOW_NORMAL);
	cv::imshow("Homography", homography_frame_);

	if(!init_imageCallback_)
	{
		cv::namedWindow("testCrossing", cv::WINDOW_NORMAL);
		cv::imshow("testCrossing", testCrossing);
	}
	init_imageCallback_ = false;

	cv::waitKey(1);
}

void LaneDetector::quickSortLinesY(int left, int right)
{
	int i = left;
	int j = right;
	int y = lanes_vector_[(left + right) / 2][0].y;
	do
	{
		while (lanes_vector_[i][0].y > y)
			i++;

		while (lanes_vector_[j][0].y < y)
			j--;

		if (i <= j)
		{
			lanes_vector_[i].swap(lanes_vector_[j]);

			i++;
			j--;
		}
	} while (i <= j);

	if (left < j)
		LaneDetector::quickSortLinesY(left, j);

	if (right > i)
		LaneDetector::quickSortLinesY(i, right);
}

void LaneDetector::quickSortPointsY(std::vector<cv::Point> &vector_in, int left, int right)
{
	int i = left;
	int j = right;
	int y = vector_in[(left + right) / 2].y;
	do
	{
		while (vector_in[i].y > y)
			i++;

		while (vector_in[j].y < y)
			j--;

		if (i <= j)
		{
			cv::Point temp = vector_in[i];
			vector_in[i] = vector_in[j];
			vector_in[j] = temp;

			i++;
			j--;
		}
	} while (i <= j);

	if (left < j)
		LaneDetector::quickSortPointsY(vector_in, left, j);

	if (right > i)
		LaneDetector::quickSortPointsY(vector_in, i, right);
}

void LaneDetector::mergeMiddleLane()
{
	for (int i = 0; i < lanes_vector_.size(); i++)
	{
		quickSortPointsY(lanes_vector_[i], 0, lanes_vector_[i].size() - 1);
	}
	quickSortLinesY(0, lanes_vector_.size() - 1);

	for (int i = 0; i < lanes_vector_.size(); i++)
	{
		float a, b;
		for (int j = i + 1; j < lanes_vector_.size(); j++)
		{
			if(lanes_vector_[i][lanes_vector_[i].size() - 1].x - lanes_vector_[i][0].x != 0)
				a = (lanes_vector_[i][0].y - lanes_vector_[i][lanes_vector_[i].size() - 1].y) / float((lanes_vector_[i][0].x - lanes_vector_[i][lanes_vector_[i].size() - 1].x));
			else
				a = 999999;
			b = lanes_vector_[i][0].y - a * lanes_vector_[i][0].x;
			float distance = std::abs(a * lanes_vector_[j][0].x - lanes_vector_[j][0].y + b) / sqrtf(a * a + 1);
			float distance2 = getDistance(lanes_vector_[j][0], lanes_vector_[i][lanes_vector_[i].size() - 1]);

			if (distance < max_mid_line_distance_ && distance2 < max_mid_line_gap_)
			{
				lanes_vector_[i].insert(lanes_vector_[i].end(), lanes_vector_[j].begin(), lanes_vector_[j].end());
				lanes_vector_.erase(lanes_vector_.begin() + j);
				j--;
			}
		}
	}
}

float LaneDetector::getDistance(cv::Point p1, cv::Point p2)
{
	float dx = float(p1.x - p2.x);
	float dy = float(p1.y - p2.y);
	return sqrtf(dx * dx + dy * dy);
}

void LaneDetector::recognizeLines()
{
	float min = current_frame_.cols;
	int min_index = -1;
	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		float aprox_y = getAproxY(last_left_coeff_, lanes_vector_[i][0].x);
		if(std::abs(aprox_y - lanes_vector_[i][0].y) < min)
		{
			min = std::abs(aprox_y - lanes_vector_[i][0].y);
			min_index = i;
		}
	}
	if(min < max_delta_y_lane_)
		left_line_index_ = min_index;
	else
		left_line_index_ = -1;

	min = current_frame_.cols;
	min_index = -1;
	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		float aprox_y = getAproxY(last_middle_coeff_, lanes_vector_[i][0].x);
		if(std::abs(aprox_y - lanes_vector_[i][0].y) < min)
		{
			min = std::abs(aprox_y - lanes_vector_[i][0].y);
			min_index = i;
		}
	}
	if(min < max_delta_y_lane_)
		center_line_index_ = min_index;
	else
		center_line_index_ = -1;

	min = current_frame_.cols;
	min_index = -1;
	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		float aprox_y = getAproxY(last_right_coeff_, lanes_vector_[i][0].x);
		if(std::abs(aprox_y - lanes_vector_[i][0].y) < min)
		{
			min = std::abs(aprox_y - lanes_vector_[i][0].y);
			min_index = i;
		}
	}
	if(min < max_delta_y_lane_)
		right_line_index_ = min_index;
	else
		right_line_index_ = -1;
}

void LaneDetector::publishMarkings()
{
	ROS_INFO("coeff0: %.3f    coeff1: %.3f     coeff2: %.3f ",middle_coeff_[0],middle_coeff_[1],middle_coeff_[2]);
	selfie_msgs::RoadMarkings road_markings;
	road_markings.header.stamp = ros::Time::now();
	road_markings.header.frame_id = "road_markings";
	for (int i = 0; i < left_coeff_.size(); i++)
	{
		road_markings.left_line.push_back(left_coeff_[i]);
		road_markings.right_line.push_back(right_coeff_[i]);
		road_markings.center_line.push_back(middle_coeff_[i]);
	}
	lanes_pub_.publish(road_markings);
}

void LaneDetector::printInfoParams()
{
    ROS_INFO("binary_treshold: %.3f",binary_treshold_);
    ROS_INFO("max_mid_line_gap: %.3f",max_mid_line_gap_);

    ROS_INFO("visualize: %d\n",visualize_);
}

void LaneDetector::dynamicMask(cv::Mat &input_frame, cv::Mat &output_frame, std::vector<std::vector<cv::Point> > lanes_vector_last_frame)
{
	dynamic_mask_ = cv::Mat::zeros(cv::Size(input_frame.cols, input_frame.rows), CV_8UC1);
	int length;
	int offset_right = 20;
	int offset_left = 20;
	output_frame = input_frame.clone();
	if(right_line_index_ == -1)
		offset_right = 70;
	if(left_line_index_ == -1)
		offset_left = 70;

	int l, k;
	length = lanes_vector_last_frame[0].size() + lanes_vector_last_frame[2].size();
	cv::Point points[length];
	for (l = 0; l < lanes_vector_last_frame[0].size(); l++)
	{
		points[l] = cv::Point(lanes_vector_last_frame[0][l].x - offset_left, lanes_vector_last_frame[0][l].y);
	}
	for (k = lanes_vector_last_frame[2].size() - 1; k >= 0; k--)
	{
		points[l] = cv::Point(lanes_vector_last_frame[2][k].x + offset_right, lanes_vector_last_frame[2][k].y);
		l++;
	}
	cv::fillConvexPoly(dynamic_mask_, points, length, cv::Scalar(255, 255, 255));
	cv::bitwise_and(input_frame, dynamic_mask_, output_frame);
}

void LaneDetector::crossingLane(cv::Mat &input_frame, cv::Mat &output_frame, std::vector<std::vector<cv::Point> > lanes_vector_last_frame)
{
	crossing_ROI_ = cv::Mat::zeros(cv::Size(input_frame.cols, input_frame.rows), CV_8UC1);
	output_frame = input_frame.clone();
	int offset_center = 20;
	int offset_right = 15;
	int length;

	int l, k, m = 0;
	if(right_line_index_ == -1)
		return;
	length = lanes_vector_last_frame[2].size() + lanes_vector_last_frame[1].size();
	cv::Point points[length];
	for (l = 0; l < lanes_vector_last_frame[2].size(); l++)
	{
		points[m] = cv::Point(lanes_vector_last_frame[2][l].x - offset_right, lanes_vector_last_frame[2][l].y);
		m++;
	}
	for (k = lanes_vector_last_frame[1].size() - 1; k >= 0; k--)
	{
		points[m] = cv::Point(lanes_vector_last_frame[1][k].x + offset_center, lanes_vector_last_frame[1][k].y);
		m++;
	}

	for (int n = 0; n < length - 1; n++)
	{
		cv::line(output_frame, points[n], points[n + 1], cv::Scalar(0, 0, 255), 2);
	}

	cv::fillConvexPoly(crossing_ROI_, points, length, cv::Scalar(255, 0, 0));

	cv::bitwise_and(input_frame, crossing_ROI_, output_frame);
}

void LaneDetector::filterSmallLines()
{
	for(int i = 0;i < lanes_vector_.size(); i++)
	{
		float sum = arcLength(lanes_vector_[i], false);
		if(sum < min_length_search_line_)
		{
			lanes_vector_.erase(lanes_vector_.begin() + i);
			i--;
		}
	}
}

void LaneDetector::convertCoordinates()
{
	int temp_x, temp_y;
	for(int i = 0; i < lanes_vector_.size(); i++)
		quickSortPointsY(lanes_vector_[i], 0, lanes_vector_.size() - 1);

	quickSortLinesY(0, lanes_vector_.size() - 1);

	for(int i = 0; i < lanes_vector_.size(); i++)
		for(int j = 0; j < lanes_vector_[i].size(); j++)
		{
			temp_x = lanes_vector_[i][j].x;
			temp_y = lanes_vector_[i][j].y;
			lanes_vector_[i][j].x = homography_frame_.rows - temp_y;
			lanes_vector_[i][j].y = homography_frame_.cols / 2 - temp_x;
		}
}

float LaneDetector::getAproxY(std::vector<float> coeff, float x)
{
	size_t nDegree = coeff.size();

    float nY = 0;
    float nXdouble = 1;
    float nX = x;

    for ( size_t j = 0; j < nDegree; j++ )
    {
        // multiply current x by a coefficient
        nY += coeff[j] * nXdouble;
        // power up the X
        nXdouble *= nX;
    }
    return nY;
}

void LaneDetector::calcValuesForMasks()
{
	aprox_lines_frame_coordinate_[0].clear();
	aprox_lines_frame_coordinate_[1].clear();
	aprox_lines_frame_coordinate_[2].clear();

	cv::Point p;
	float increment = 10;
	for(float i = 0; i < homography_frame_.rows; i += increment)
	{
		p.y = homography_frame_.rows - i;
		p.x = homography_frame_.cols / 2 - getAproxY(left_coeff_, i);
		//if(p.x < 0 || p.x > current_frame_.cols)
			//break;
		aprox_lines_frame_coordinate_[0].push_back(p);
	}
	for(float i = 0; i < homography_frame_.rows; i += increment)
	{
		p.y = homography_frame_.rows - i;
		p.x = homography_frame_.cols / 2 - getAproxY(middle_coeff_, i);
		//if(p.x < 0 || p.x > current_frame_.cols)
			//break;
		aprox_lines_frame_coordinate_[1].push_back(p);
	}
	for(float i = 0; i < homography_frame_.rows; i += increment)
	{
		p.y = homography_frame_.rows - i;
		p.x = homography_frame_.cols / 2 - getAproxY(right_coeff_, i);
		//if(p.x < 0 || p.x > current_frame_.cols)
			//break;
		aprox_lines_frame_coordinate_[2].push_back(p);
	}
}

void LaneDetector::initRecognizeLines()
{
	float min = homography_frame_.cols;
	int min_index = -1;
	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		if(std::abs(nominal_center_line_Y_ - lanes_vector_[i][0].y) < min && lanes_vector_[i][0].x < homography_frame_.rows / 2)
		{
			min = std::abs(nominal_center_line_Y_ - lanes_vector_[i][0].y);
			min_index = i;
		}
	}
	center_line_index_ = min_index;

	min = homography_frame_.cols;
	min_index = -1;
	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		if(lanes_vector_[i][0].y < lanes_vector_[center_line_index_][0].y && lanes_vector_[i][0].x < homography_frame_.rows / 2)
			if(std::abs(lanes_vector_[i][0].y - lanes_vector_[center_line_index_][0].y) < min)
			{
				min = std::abs(lanes_vector_[i][0].y - lanes_vector_[center_line_index_][0].y);
				min_index = i;
			}
	}
	right_line_index_ = min_index;

	min = homography_frame_.cols;
	min_index = -1;
	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		if(lanes_vector_[i][0].y > lanes_vector_[center_line_index_][0].y && lanes_vector_[i][0].x < homography_frame_.rows / 2)
			if(std::abs(lanes_vector_[i][0].y - lanes_vector_[center_line_index_][0].y) < min)
			{
				min = std::abs(lanes_vector_[i][0].y - lanes_vector_[center_line_index_][0].y);
				min_index = i;
			}
	}
	left_line_index_ = min_index;
}

void LaneDetector::pointsRVIZVisualization()
{
    geometry_msgs::Point32 point;
	point.z = 0;
	points_cloud_.points.clear();
	points_cloud_.header.frame_id = "laser";

	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		for(int j = 0; j < lanes_vector_[i].size(); j++)
		{
			point.x = lanes_vector_[i][j].x;
			point.y = lanes_vector_[i][j].y;
			points_cloud_.points.push_back(point);
		}
	}

	points_cloud_pub_.publish(points_cloud_);
}

void LaneDetector::aproxVisualization()
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "line";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.lifetime = ros::Duration();

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.scale.x = 2;
    marker.scale.y = 2;

    geometry_msgs::Point marker_point;
	marker_point.z = 0;

	float increment = 5;
	for(float i = 0; i < current_frame_.rows; i += increment)
	{
		marker_point.x = i;
		marker_point.y = getAproxY(right_coeff_, i);
		marker.points.push_back(marker_point);
	}
		for(float i = 0; i < current_frame_.rows; i += increment)
	{
		marker_point.x = i;
		marker_point.y = getAproxY(middle_coeff_, i);
		marker.points.push_back(marker_point);
	}
	aprox_visualization_pub_.publish(marker);
}

void LaneDetector::filterPoints()
{
	float max = 0.5;
	if(left_line_index_ != -1)
	{
		for(int i = 0; i < lanes_vector_[left_line_index_].size() - 1; i++)
		{
			if(lanes_vector_[left_line_index_][i].x == lanes_vector_[left_line_index_][i + 1].x)
			{
				lanes_vector_[left_line_index_].erase(lanes_vector_[left_line_index_].begin() + i + 1);
				i--;
				continue;
			}
			float slope = (lanes_vector_[left_line_index_][i + 1].y - lanes_vector_[left_line_index_][i].y) / float(lanes_vector_[left_line_index_][i + 1].x - lanes_vector_[left_line_index_][i].x);
			float aprox_slope = 2 * last_left_coeff_[2] * lanes_vector_[left_line_index_][i].x + last_left_coeff_[1];
			if(std::abs(slope - aprox_slope) > max)
			{
				lanes_vector_[left_line_index_].erase(lanes_vector_[left_line_index_].begin() + i + 1);
				i--;
			}
		}
	}

	if(center_line_index_ != -1)
	{
		for(int i = 0; i < lanes_vector_[center_line_index_].size() - 1; i++)
		{
			if(lanes_vector_[center_line_index_][i].x == lanes_vector_[center_line_index_][i + 1].x)
			{
				lanes_vector_[center_line_index_].erase(lanes_vector_[center_line_index_].begin() + i + 1);
				i--;
				continue;
			}
			float slope = (lanes_vector_[center_line_index_][i + 1].y - lanes_vector_[center_line_index_][i].y) / float(lanes_vector_[center_line_index_][i + 1].x - lanes_vector_[center_line_index_][i].x);
			float aprox_slope = 2 * last_middle_coeff_[2] * lanes_vector_[center_line_index_][i].x + last_middle_coeff_[1];
			if(std::abs(slope - aprox_slope) > max)
			{
				lanes_vector_[center_line_index_].erase(lanes_vector_[center_line_index_].begin() + i + 1);
				i--;
			}
		}
	}

	if(right_line_index_ != -1)
	{
		for(int i = 0; i < lanes_vector_[right_line_index_].size() - 1; i++)
		{
			if(lanes_vector_[right_line_index_][i].x == lanes_vector_[right_line_index_][i + 1].x)
			{
				lanes_vector_[right_line_index_].erase(lanes_vector_[right_line_index_].begin() + i + 1);
				i--;
				continue;
			}
			float slope = (lanes_vector_[right_line_index_][i + 1].y - lanes_vector_[right_line_index_][i].y) / float(lanes_vector_[right_line_index_][i + 1].x - lanes_vector_[right_line_index_][i].x);
			float aprox_slope = 2 * last_right_coeff_[2] * lanes_vector_[right_line_index_][i].x + last_right_coeff_[1];
			if(std::abs(slope - aprox_slope) > max)
			{
				lanes_vector_[right_line_index_].erase(lanes_vector_[right_line_index_].begin() + i + 1);
				i--;
			}
		}
	}
}


void LaneDetector::linesApproximation(std::vector<std::vector<cv::Point> > lanes_vector)
{
	int poly_nDegree = 2;
	std::cout << "appr start" << std::endl;
	left_coeff_.clear();
	middle_coeff_.clear();
	right_coeff_.clear();

	// vectors length count
	const double min_length = 150;
	bool shrt_left = false, shrt_right = false, shrt_middle = false;
	double left_length = 0, right_length = 0, middle_length = 0;

	std::cout << "left_index" << left_line_index_ << std::endl;
	std::cout << "middle_index " << center_line_index_ << std::endl;
	std::cout << "right_index " << right_line_index_ << std::endl;

	if(left_line_index_ != -1)
		left_length = cv::arcLength(lanes_vector[left_line_index_], false);
	else
		left_length = 0;
	if(center_line_index_ != -1)
		middle_length = cv::arcLength(lanes_vector[center_line_index_], false);
	else
		middle_length = 0;
	if(right_line_index_ != -1)
		right_length = cv::arcLength(lanes_vector[right_line_index_], false);
	else
		right_length = 0;

	std::cout << "left_length " << left_length << std::endl;
	std::cout << "middle_length " << middle_length << std::endl;
	std::cout << "right_length " << right_length << std::endl;

	int suitable_lines = 3;

	if(left_length < min_length)
	{
		shrt_left = true;
		suitable_lines--;
	}

	if(right_length < min_length)
	{
		shrt_right = true;
		suitable_lines--;
	}

	if(middle_length < min_length)
	{
		shrt_middle = true;
		suitable_lines--;
	}

	switch(suitable_lines)
	{
		case 3:
		ROS_INFO("l+  c+  r+");
		left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);
		center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
		right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);

		left_line_poly_.polyfit(poly_nDegree);
		center_line_poly_.polyfit(poly_nDegree);
		right_line_poly_.polyfit(poly_nDegree);

		left_coeff_ = left_line_poly_.coeff;
		middle_coeff_ = center_line_poly_.coeff;
		right_coeff_= right_line_poly_.coeff;
		break;

		case 2:
		if(shrt_left)
		{
			center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
			right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);

			center_line_poly_.polyfit(poly_nDegree);
			right_line_poly_.polyfit(poly_nDegree);

			middle_coeff_ = center_line_poly_.coeff;
			right_coeff_= right_line_poly_.coeff;
			if(left_line_index_ == -1)
			{
				ROS_INFO("l-  c+  r+");
				left_coeff_ = middle_coeff_;
				left_coeff_[0] += middle_coeff_[0] - right_coeff_[0];
			}
			else
			{
				ROS_INFO("l/  c+  r+");
				left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);
				left_line_poly_.adjust(center_line_poly_, lanes_vector[left_line_index_].size());
				left_coeff_ = left_line_poly_.coeff;
			}
		}
		else if(shrt_right)
		{
			center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
			left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);

			center_line_poly_.polyfit(poly_nDegree);
			left_line_poly_.polyfit(poly_nDegree);

			middle_coeff_ = center_line_poly_.coeff;
			left_coeff_= left_line_poly_.coeff;
			if(right_line_index_ == -1)
			{
				ROS_INFO("l+  c+  r-");
				right_coeff_ = middle_coeff_;
				right_coeff_[0] -= left_coeff_[0] - middle_coeff_[0];
			}
			else
			{
				ROS_INFO("l+  c+  r/");
				right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);
				right_line_poly_.adjust(center_line_poly_, lanes_vector[right_line_index_].size());
				right_coeff_ = right_line_poly_.coeff;
			}
		}
		else if(shrt_middle)
		{
			right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);
			left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);

			right_line_poly_.polyfit(poly_nDegree);
			left_line_poly_.polyfit(poly_nDegree);

			right_coeff_ = right_line_poly_.coeff;
			left_coeff_= left_line_poly_.coeff;
			if(center_line_index_ == -1)
			{
				ROS_INFO("l+  c-  r+");
				middle_coeff_ = right_coeff_;
				middle_coeff_[0] += (left_coeff_[0] - right_coeff_[0]) / 2;
			}
			else
			{
				ROS_INFO("l+  c/  r+");
				center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
				center_line_poly_.adjust(right_line_poly_, lanes_vector[center_line_index_].size());
				middle_coeff_ = center_line_poly_.coeff;
			}
		}
		break;

		case 1:
		if(!shrt_right)
		{
			right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);
			right_line_poly_.polyfit(poly_nDegree);
			right_coeff_ = right_line_poly_.coeff;
			if(center_line_index_ != -1 || left_line_index_ != -1)
			{
				if(center_line_index_ != -1 && left_line_index_ != -1)
				{
					ROS_INFO("l/  c/  r+");
					center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
					center_line_poly_.adjust(right_line_poly_, lanes_vector[center_line_index_].size());
					middle_coeff_ = center_line_poly_.coeff;

					left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);
					left_line_poly_.adjust(right_line_poly_, lanes_vector[left_line_index_].size());
					left_coeff_ = left_line_poly_.coeff;
				}
				else if(center_line_index_ != -1)
				{
					ROS_INFO("l-  c/  r+");
					center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
					center_line_poly_.adjust(right_line_poly_, lanes_vector[center_line_index_].size());
					middle_coeff_ = center_line_poly_.coeff;

					left_coeff_ = middle_coeff_;
					left_coeff_[0] += middle_coeff_[0] - right_coeff_[0];
				}
				else
				{
					ROS_INFO("l/  c-  r+");
					left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);
					left_line_poly_.adjust(right_line_poly_, lanes_vector[left_line_index_].size());
					left_coeff_ = left_line_poly_.coeff;

					middle_coeff_ = right_coeff_;
					middle_coeff_[0] += (left_coeff_[0] - right_coeff_[0]) / 2;
				}
			}
			else
			{
				ROS_INFO("l-  c-  r+");
				middle_coeff_ = last_middle_coeff_;
				left_coeff_ = last_left_coeff_;
			}
		}

		else if(!shrt_left)
		{
			left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);
			left_line_poly_.polyfit(poly_nDegree);
			left_coeff_ = left_line_poly_.coeff;
			if(center_line_index_ != -1 || right_line_index_ != -1)
			{
				if(center_line_index_ != -1 && right_line_index_ != -1)
				{
					ROS_INFO("l+  c/  r/");
					center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
					center_line_poly_.adjust(left_line_poly_, lanes_vector[center_line_index_].size());
					middle_coeff_ = center_line_poly_.coeff;

					right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);
					right_line_poly_.adjust(left_line_poly_, lanes_vector[right_line_index_].size());
					right_coeff_ = right_line_poly_.coeff;
				}
				else if(center_line_index_ != -1)
				{
					ROS_INFO("l+  c/  r-");
					center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
					center_line_poly_.adjust(left_line_poly_, lanes_vector[center_line_index_].size());
					middle_coeff_ = center_line_poly_.coeff;

					right_coeff_ = middle_coeff_;
					right_coeff_[0] -= left_coeff_[0] - middle_coeff_[0];
				}
				else
				{
					ROS_INFO("l+  c-  r/");
					right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);
					right_line_poly_.adjust(left_line_poly_, lanes_vector[right_line_index_].size());
					right_coeff_ = right_line_poly_.coeff;

					middle_coeff_ = right_coeff_;
					middle_coeff_[0] += (left_coeff_[0] - right_coeff_[0]) / 2;
				}
			}
			else
			{
				ROS_INFO("l+  c-  r-");
				middle_coeff_ = last_middle_coeff_;
				right_coeff_ = last_right_coeff_;
			}
		}

		else if(!shrt_middle)
		{
			center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
			center_line_poly_.polyfit(poly_nDegree);
			middle_coeff_ = center_line_poly_.coeff;
			if(left_line_index_ != -1 || right_line_index_ != -1)
			{
				if(left_line_index_ != -1 && right_line_index_ != -1)
				{
					ROS_INFO("l/  c+  r/");
					left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);
					left_line_poly_.adjust(center_line_poly_, lanes_vector[left_line_index_].size());
					left_coeff_ = left_line_poly_.coeff;

					right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);
					right_line_poly_.adjust(center_line_poly_, lanes_vector[right_line_index_].size());
					right_coeff_ = right_line_poly_.coeff;
				}
				else if(left_line_index_ != -1)
				{
					ROS_INFO("l/  c+  r-");
					left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);
					left_line_poly_.adjust(center_line_poly_, lanes_vector[left_line_index_].size());
					left_coeff_ = left_line_poly_.coeff;

					right_coeff_ = middle_coeff_;
					right_coeff_[0] -= left_coeff_[0] - middle_coeff_[0];
				}
				else
				{
					ROS_INFO("l-  c+  r/");
					right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);
					right_line_poly_.adjust(center_line_poly_, lanes_vector[right_line_index_].size());
					right_coeff_ = right_line_poly_.coeff;

					left_coeff_ = middle_coeff_;
					left_coeff_[0] += middle_coeff_[0] - right_coeff_[0];
				}
			}
			else
			{
				ROS_INFO("l-  c+  r-");
				left_coeff_ = last_left_coeff_;
				right_coeff_ = last_right_coeff_;
			}
		}
		break;

		case 0:
		ROS_INFO("l-  c-  r-");
		left_coeff_ = last_left_coeff_;
		right_coeff_ = last_right_coeff_;
		middle_coeff_ = last_middle_coeff_;
		break;
	}

	last_left_coeff_.clear();
	last_middle_coeff_.clear();
	last_right_coeff_.clear();

	last_left_coeff_ = left_coeff_;
	last_middle_coeff_ = middle_coeff_;
	last_right_coeff_ = right_coeff_;
}

void LaneDetector::lanesVectorVisualization(cv::Mat &visualization_frame)
{
	visualization_frame = cv::Mat::zeros(visualization_frame.size(), CV_8UC3);
	if(lanes_vector_.empty())
		return;
	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		for(int j = 0; j < lanes_vector_[i].size(); j++)
		{
			cv::circle(visualization_frame, lanes_vector_[i][j], 2, cv::Scalar(255, 0, 0), CV_FILLED, cv::LINE_AA);
		}
	}

	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		for(int j = 1; j < lanes_vector_[i].size(); j++)
		{
			cv::line(visualization_frame, lanes_vector_[i][j],lanes_vector_[i][j - 1], cv::Scalar(255, 255, 0), 1);
		}
	}
}
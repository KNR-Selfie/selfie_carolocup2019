#include <selfie_perception/lanedetector.h>

#define PI 3.1415926

static int Acc_slider = 1;
static int Acc_value = 1;
static int Acc_filt = 5;
static int Acc_filt_slider = 40;
static int Alpha_ = 12;
static int F_ = 500, Dist_ = 500;

LaneDetector::LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : 
	nh_(nh),
	pnh_(pnh),
	it_(nh),
	binary_treshold_(195),
	mask_initialized_(false),
	visualize_(true),
	max_mid_line_gap_(155),
	max_mid_line_X_gap_ (80),

	kernel_v_(),
	current_frame_(),
	gray_frame_(),
	binary_frame_(),
	mask_(),
	canny_frame_(),
	visualization_frame_(),
	homography_frame_()
{
	lanes_pub_ =  nh_.advertise<selfie_msgs::RoadMarkings>("road_markings", 10);
}

LaneDetector::~LaneDetector()
{
	cv::destroyAllWindows();
}

bool LaneDetector::init()
{
	kernel_v_ = cv::Mat(1, 3, CV_32F);
	kernel_v_.at<float>(0, 0) = -1;
	kernel_v_.at<float>(0, 1) = 0;
	kernel_v_.at<float>(0, 2) = 1;

	pnh_.getParam("binary_treshold",binary_treshold_);
	pnh_.getParam("visualize",visualize_);
    pnh_.getParam("max_mid_line_gap",max_mid_line_gap_);
    pnh_.getParam("max_mid_line_X_gap",max_mid_line_X_gap_);

	image_sub_ = it_.subscribe("/image_raw", 1, &LaneDetector::imageCallback, this);

	printInfoParams();
	return true;
}

void LaneDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		current_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (!mask_initialized_)
	{
		mask_ = cv::Mat::zeros(cv::Size(current_frame_.cols, current_frame_.rows), CV_8UC1);
		cv::Point points[4] =
			{
				cv::Point(60, current_frame_.rows),
				cv::Point(current_frame_.cols - 60, current_frame_.rows),
				cv::Point(current_frame_.cols - 60, current_frame_.rows / 3),
				cv::Point(60, current_frame_.rows / 3)};
		cv::fillConvexPoly(mask_, points, 4, cv::Scalar(255, 0, 0));
		mask_initialized_ = true;
	}
	homography(current_frame_, homography_frame_);
	cv::cvtColor(homography_frame_, gray_frame_, cv::COLOR_BGR2GRAY);
	cv::threshold(gray_frame_, binary_frame_, binary_treshold_, 255, cv::THRESH_BINARY);
	cv::bitwise_and(binary_frame_, mask_, canny_frame_);
	cv::medianBlur(binary_frame_, canny_frame_, 5);
	cv::filter2D(binary_frame_, canny_frame_, -1, kernel_v_, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

	detectLines(canny_frame_, lanes_vector_);
	if(!lanes_vector_.empty())
	{
		mergeMiddleLane();
		recognizeLines();
		publishMarkings();
	}

	if (visualize_)
	{
		visualization_frame_.rows = current_frame_.rows;
		visualization_frame_.cols = current_frame_.cols;
		drawPoints(visualization_frame_);
		openCVVisualization();
	}
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
	if(!lanes_vector_[2].empty())
		for (int i = 0; i < lanes_vector_[2].size(); i++)
		{
			cv::circle(frame, lanes_vector_[2][i], 3, cv::Scalar(255, 0, 0), CV_FILLED, cv::LINE_AA);
		}
	if(!lanes_vector_[0].empty())
		for (int i = 0; i < lanes_vector_[0].size(); i++)
		{
			cv::circle(frame, lanes_vector_[0][i], 3, cv::Scalar(0, 0, 255), CV_FILLED, cv::LINE_AA);
		}
	if(!lanes_vector_[1].empty())
		for (int i = 0; i < lanes_vector_[1].size(); i++)
		{
			cv::circle(frame, lanes_vector_[1][i], 3, cv::Scalar(0, 255, 0), CV_FILLED, cv::LINE_AA);
		}
}

void LaneDetector::homography(cv::Mat input_frame, cv::Mat &homography_frame)
{
	resize(input_frame, input_frame, cv::Size(640, 480));

	double focal_lenth, dist, alpha;

	alpha = ((double)Alpha_ - 90) * PI / 180;
	focal_lenth = (double)F_;
	dist = (double)Dist_;

	cv::Size image_size = input_frame.size();
	double w = (double)image_size.width, h = (double)image_size.height;

	// Matrix 2D -> 3D
	cv::Mat A1 = (cv::Mat_<float>(4, 3) <<
		1, 0, -w / 2,
		0, 1, -h / 2,
		0, 0, 0,
		0, 0, 1);
	
	// Rotation matrix
	cv::Mat RX = (cv::Mat_<float>(4, 4) <<
		1, 0, 0, 0,
		0, cos(alpha), -sin(alpha), 0,
		0, sin(alpha), cos(alpha), 0,
		0, 0, 0, 1);
	
	cv::Mat R = RX;

	// Translation matrix
	cv::Mat T = (cv::Mat_<float>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, dist,
		0, 0, 0, 1);

	// Intrinsic matrix
	cv::Mat K = (cv::Mat_<float>(3, 4) <<
		focal_lenth, 0, w / 2, 0,
		0, focal_lenth, h / 2, 0,
		0, 0, 1, 0
		);

	cv::Mat transformationMat = K * (T * (R * A1));

	cv::warpPerspective(input_frame, homography_frame, transformationMat, image_size, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
}

void LaneDetector::openCVVisualization()
{
	//cv::namedWindow("Raw image", cv::WINDOW_NORMAL);
	//cv::imshow("Raw image", current_frame_);

	cv::namedWindow("Binarization", cv::WINDOW_NORMAL);
	cv::imshow("Binarization", binary_frame_);

	//cv::namedWindow("Canny", cv::WINDOW_NORMAL);
	//cv::imshow("Canny", canny_frame_);

	cv::namedWindow("Homography", cv::WINDOW_NORMAL);
	cv::imshow("Homography", homography_frame_);

	cv::namedWindow("Output", cv::WINDOW_NORMAL);
	cv::imshow("Output", visualization_frame_);
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
		for (int j = i + 1; j < lanes_vector_.size(); j++)
		{
			float distance = getDistance(lanes_vector_[j][0], lanes_vector_[i][lanes_vector_[i].size() - 1]);
			float distance_x = std::abs(lanes_vector_[j][0].x - lanes_vector_[i][lanes_vector_[i].size() - 1].x);

			if (distance < max_mid_line_gap_ && distance_x < max_mid_line_X_gap_)
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
	std::vector<cv::Point> empty;
	empty.clear();
	switch (lanes_vector_.size())
	{
	case 1:
		lanes_vector_.insert(lanes_vector_.begin(), empty);
		lanes_vector_.insert(lanes_vector_.begin(), empty);
		break;
	case 2:
		if (lanes_vector_[0][0].x > lanes_vector_[1][0].x)
		{
			lanes_vector_[0].swap(lanes_vector_[1]);
			lanes_vector_.insert(lanes_vector_.begin(), empty);
		}
		else
		{
			lanes_vector_.insert(lanes_vector_.begin(), empty);
		}
		break;
	case 3:
		if (lanes_vector_[1][0].x > lanes_vector_[0][0].x)
		{
			if (lanes_vector_[0][0].x > lanes_vector_[2][0].x)
			{
				lanes_vector_[0].swap(lanes_vector_[2]);
				lanes_vector_[1].swap(lanes_vector_[2]);
			}
			else if (lanes_vector_[2][0].x > lanes_vector_[1][0].x)
			{
				;
			}
			else
			{
				lanes_vector_[2].swap(lanes_vector_[1]);
			}
		}
		else if (lanes_vector_[1][0].x > lanes_vector_[2][0].x)
		{
			lanes_vector_[2].swap(lanes_vector_[0]);
		}
		else if (lanes_vector_[0][0].x > lanes_vector_[2][0].x)
		{
			lanes_vector_[1].swap(lanes_vector_[0]);
			lanes_vector_[1].swap(lanes_vector_[2]);
		}
		else
		{
			lanes_vector_[1].swap(lanes_vector_[0]);
		}
		break;
	default:
		lanes_vector_ = lanes_vector_last_frame;
		break;
	}
}

void LaneDetector::publishMarkings()
{
	lanes_vector_last_frame = lanes_vector_;
	selfie_msgs::RoadMarkings road_markings;
	road_markings.header.stamp = ros::Time::now();
	road_markings.header.frame_id = "road_markings";
	geometry_msgs::Point p;
	p.z = 0;
	if(!lanes_vector_[2].empty())
		for (int i = 0; i < lanes_vector_[2].size(); i++)
		{
			p.x = lanes_vector_[2][i].x;
			p.y = lanes_vector_[2][i].y;
			road_markings.right_line.push_back(p);
		}
	if(!lanes_vector_[0].empty())
		for (int i = 0; i < lanes_vector_[0].size(); i++)
		{
			p.x = lanes_vector_[0][i].x;
			p.y = lanes_vector_[0][i].y;
			road_markings.left_line.push_back(p);
		}
	if(!lanes_vector_[1].empty())
		for (int i = 0; i < lanes_vector_[1].size(); i++)
		{
			p.x = lanes_vector_[1][i].x;
			p.y = lanes_vector_[1][i].y;
			road_markings.center_line.push_back(p);
		}
	lanes_pub_.publish(road_markings);
}

void LaneDetector::printInfoParams()
{
    ROS_INFO("binary_treshold: %.3f",binary_treshold_);
    ROS_INFO("max_mid_line_gap: %.3f",max_mid_line_gap_);
    ROS_INFO("max_mid_line_X_gap: %.3f\n",max_mid_line_X_gap_);

    ROS_INFO("visualize: %d\n",visualize_);
}
#include <selfie_perception/lanedetector.h>

#define PI 3.1415926

static int Acc_slider = 1;
static int Acc_value = 1;
static int Acc_filt = 5;
static int Acc_filt_slider = 40;
static int Alpha_ = 12;
static int F_ = 500, Dist_ = 500;

poly left_line_poly_;
poly center_line_poly_;
poly right_line_poly_;

LaneDetector::LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : 
	nh_(nh),
	pnh_(pnh),
	it_(nh),
	binary_treshold_(195),
	visualize_(true),
	max_mid_line_gap_(200),
	max_mid_line_distance_(50),
	init_imageCallback_(true),

	min_length_search_line_(30),
	min_length_lane_(67),
	max_delta_y_lane_(75),
	nominal_center_line_Y_(180),

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
	points_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("new_coordinates", 10);
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

	pnh_.getParam("binary_treshold",binary_treshold_);
	pnh_.getParam("visualize",visualize_);
    pnh_.getParam("max_mid_line_gap",max_mid_line_gap_);

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
	homography(current_frame_, homography_frame_);
	cv::cvtColor(homography_frame_, gray_frame_, cv::COLOR_BGR2GRAY);
	cv::threshold(gray_frame_, binary_frame_, binary_treshold_, 255, cv::THRESH_BINARY);

	cv::medianBlur(binary_frame_, binary_frame_, 5);
	cv::filter2D(binary_frame_, canny_frame_, -1, kernel_v_, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

	detectLines(canny_frame_, lanes_vector_);
	filterSmallLines();

	if(!lanes_vector_.empty())
	{
		mergeMiddleLane();
		convertCoordinates();
		pointsRVIZVisualization();
		if(init_imageCallback_)
		{
			initRecognizeLines();
			init_imageCallback_ = false;
		}
		else
			recognizeLines();
		ROS_INFO("left: %d   center %d   right %d", left_line_index_, center_line_index_, right_line_index_);
		linesApproximation(lanes_vector_);
		calcValuesForMasks();
	}

	//publishMarkings();

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

/*void LaneDetector::publishMarkings()
{
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
}*/

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
	int offset = 20;
	output_frame = input_frame.clone();

	int l, k;
	length = lanes_vector_last_frame[0].size() + lanes_vector_last_frame[2].size();
	cv::Point points[length];
	for (l = 0; l < lanes_vector_last_frame[0].size(); l++)
	{
		points[l] = cv::Point(lanes_vector_last_frame[0][l].x - offset, lanes_vector_last_frame[0][l].y);
	}
	for (k = lanes_vector_last_frame[2].size() - 1; k >= 0; k--)
	{
		points[l] = cv::Point(lanes_vector_last_frame[2][k].x + offset, lanes_vector_last_frame[2][k].y);
		l++;
	}
	cv::fillConvexPoly(dynamic_mask_, points, length, cv::Scalar(150, 150, 0));
	cv::bitwise_and(input_frame, dynamic_mask_, output_frame);
}

void LaneDetector::crossingLane(cv::Mat &input_frame, cv::Mat &output_frame, std::vector<std::vector<cv::Point> > lanes_vector_last_frame)
{
	crossing_ROI_ = cv::Mat::zeros(cv::Size(input_frame.cols, input_frame.rows), CV_8UC1);
	output_frame = input_frame.clone();
	int offset = 20;
	int length;

	int l, k, m = 0;
	length = lanes_vector_last_frame[2].size() + lanes_vector_last_frame[1].size();
	cv::Point points[length];
	for (l = 0; l < lanes_vector_last_frame[2].size(); l++)
	{
		points[m] = cv::Point(lanes_vector_last_frame[2][l].x - offset, lanes_vector_last_frame[2][l].y);
		m++;
	}
	for (k = lanes_vector_last_frame[1].size() - 1; k >= 0; k--)
	{
		points[m] = cv::Point(lanes_vector_last_frame[1][k].x + offset, lanes_vector_last_frame[1][k].y);
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
			lanes_vector_[i][j].x = current_frame_.rows - temp_y;
			lanes_vector_[i][j].y = current_frame_.cols / 2 - temp_x;
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
	float increment = 5;
	for(float i = 0; i < current_frame_.rows; i += increment)
	{
		p.y = current_frame_.rows - i;
		p.x = current_frame_.cols / 2 - getAproxY(left_coeff_, i);
		if(p.x < 0 || p.x > current_frame_.cols)
			break;
		aprox_lines_frame_coordinate_[0].push_back(p);
	}
	for(float i = 0; i < current_frame_.rows; i += increment)
	{
		p.y = current_frame_.rows - i;
		p.x = current_frame_.cols / 2 - getAproxY(middle_coeff_, i);
		if(p.x < 0 || p.x > current_frame_.cols)
			break;
		aprox_lines_frame_coordinate_[1].push_back(p);
	}
	for(float i = 0; i < current_frame_.rows; i += increment)
	{
		p.y = current_frame_.rows - i;
		p.x = current_frame_.cols / 2 - getAproxY(right_coeff_, i);
		if(p.x < 0 || p.x > current_frame_.cols)
			break;
		aprox_lines_frame_coordinate_[2].push_back(p);
	}
}

void LaneDetector::initRecognizeLines()
{
	float min = current_frame_.cols;
	int min_index = -1;
	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		if(std::abs(nominal_center_line_Y_ - lanes_vector_[i][0].y) < min)
		{
			min = std::abs(nominal_center_line_Y_ - lanes_vector_[i][0].y);
			min_index = i;
		}
	}
	center_line_index_ = min_index;

	min = current_frame_.cols;
	min_index = -1;
	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		if(lanes_vector_[i][0].y < lanes_vector_[center_line_index_][0].y)
			if(std::abs(lanes_vector_[i][0].y - lanes_vector_[center_line_index_][0].y) < min)
			{
				min = std::abs(lanes_vector_[i][0].y - lanes_vector_[center_line_index_][0].y);
				min_index = i;
			}
	}
	right_line_index_ = min_index;

	min = current_frame_.cols;
	min_index = -1;
	for(int i = 0; i < lanes_vector_.size(); i++)
	{
		if(lanes_vector_[i][0].y > lanes_vector_[center_line_index_][0].y)
			if(std::abs(lanes_vector_[i][0].y - lanes_vector_[center_line_index_][0].y) < min)
			{
				min = std::abs(lanes_vector_[i][0].y - lanes_vector_[center_line_index_][0].y);
				min_index = i;
			}
	}
	left_line_index_ = min_index;
}

void LaneDetector::linesApproximation(std::vector<std::vector<cv::Point> > lanes_vector)
{
	left_coeff_.clear();
	middle_coeff_.clear();
	right_coeff_.clear();

	// vectors length count
	const double min_length = 80;
	bool shrt_left = false, shrt_right = false, shrt_middle = false;
	double left_length = 0, right_length = 0, middle_length = 0;

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

	if(left_length < min_length)
		shrt_left = true;
	if(middle_length < min_length)
		shrt_right = true;
	if(right_length < min_length)
		shrt_middle = true;

	// all lines
	if(!shrt_left && !shrt_middle && !shrt_right)
	{
		left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);
		center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
		right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);

		left_line_poly_.polyfit(2);
		center_line_poly_.polyfit(2);
		right_line_poly_.polyfit(2);

		left_coeff_ = left_line_poly_.coeff;
		middle_coeff_ = center_line_poly_.coeff;
		right_coeff_= right_line_poly_.coeff;		
	}
	// middle and right lines, left line too short
	else if(shrt_left && !shrt_middle && !shrt_right)
	{
		center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
		right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);

		center_line_poly_.polyfit(2);
		right_line_poly_.polyfit(2);

		middle_coeff_ = center_line_poly_.coeff;
		right_coeff_= right_line_poly_.coeff;	

		// left line exist
		if(left_line_index_ != -1)
		{
			int lf_length = lanes_vector[left_line_index_].size();
			left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);

			left_line_poly_.adjust(center_line_poly_);
			left_coeff_ = left_line_poly_.coeff;
		}
		// left line from last frame
		else
		{
			left_coeff_ = last_left_coeff_;
		}

	}
	// right line, left and center too short
	else if(shrt_left && shrt_middle && !shrt_right)
	{
		right_line_poly_.get_row_pts(lanes_vector[right_line_index_]);
		right_line_poly_.polyfit(2);
		right_coeff_ = right_line_poly_.coeff;	

		// left and center lines from last frame
		if(left_line_index_ == -1 && center_line_index_ == -1)
		{
			left_coeff_ = last_left_coeff_;
			middle_coeff_ = last_middle_coeff_;
		}
		// left line from last frame and center line exist
		else if(left_line_index_ == -1 && center_line_index_ != -1)
		{
			left_coeff_ = last_left_coeff_;

			int cn_length = lanes_vector[center_line_index_].size();
			center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);

			center_line_poly_.adjust(right_line_poly_);
			middle_coeff_ = center_line_poly_.coeff;
		}
		// left line exist and center line from last frame
		else if(left_line_index_ != -1 && center_line_index_ == -1)
		{
			middle_coeff_ = last_middle_coeff_;

			int lf_length = lanes_vector[left_line_index_].size();
			left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);

			left_line_poly_.adjust(center_line_poly_);
			left_coeff_ = left_line_poly_.coeff;
		}
		// left and center lines exist
		else
		{
			int cn_length = lanes_vector[center_line_index_].size();
			int lf_length = lanes_vector[left_line_index_].size();

			center_line_poly_.get_row_pts(lanes_vector[center_line_index_]);
			left_line_poly_.get_row_pts(lanes_vector[left_line_index_]);

			center_line_poly_.adjust(right_line_poly_);
			middle_coeff_ = center_line_poly_.coeff;
			left_line_poly_.adjust(center_line_poly_);
			left_coeff_ = left_line_poly_.coeff;
		}
	}
	// no lines, all from last frame
	else if(shrt_left && shrt_middle && shrt_right)
	{
		left_coeff_ = last_left_coeff_;
		middle_coeff_ = last_middle_coeff_;
		right_coeff_ = last_right_coeff_;
	}


	last_left_coeff_.clear();
	last_middle_coeff_.clear();
	last_right_coeff_.clear();

	last_left_coeff_ = left_coeff_;
	last_middle_coeff_ = middle_coeff_;
	last_right_coeff_ = right_coeff_;
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
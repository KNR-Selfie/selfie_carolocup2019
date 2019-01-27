#include <selfie_starting_procedure/qr_decoder.h>

Qr_decoder::Qr_decoder(const ros::NodeHandle& _pnh,const ros::NodeHandle& _nh):pnh(_pnh),nh(_nh)
{
    sub_image = nh.subscribe("image_raw",1, &Qr_decoder::imageRowCallback, this);

    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1 );

    start_counter = 0;
    search_flag = -1;
    preview_param = 0;

    pnh.getParam("preview",preview_param);

    if(preview_param > 0)
        ROS_INFO("preview mode");
    else
       ROS_INFO("no-preview mode");

    qr_tresh = 3;
    pnh.getParam("tresh",qr_tresh);

}
void Qr_decoder::decode( cv_bridge::CvImagePtr raw_image)
{
    cv_ptr = raw_image;
    cv::cvtColor(cv_ptr->image,cv_ptr->image,CV_BGR2GRAY);

    int width = cv_ptr->image.cols;
    int height = cv_ptr->image.rows;

    uchar *raw = (uchar *)cv_ptr->image.data;

    zbar::Image img(width, height, "Y800", raw, width * height);

    scanner.scan(img);
    zbar::Image::SymbolIterator symbol = img.symbol_begin();

    if(symbol == img.symbol_end()) //find empty frames
    {
        start_counter++;

        if(start_counter > qr_tresh)
        {
            ROS_INFO("GO!");
            search_flag = 0;
            cv::destroyAllWindows();
            return;
        }

        if(!preview_param)
            return;
    }

    for(symbol; symbol != img.symbol_end();++symbol)
    {
        std::cout<<symbol->get_type_name()<<" "<<symbol->get_data()<<std::endl<<std::endl; //decode result
        start_counter = 0;

        // display result
        if(preview_param)
        {
            std::vector<cv::Point> qr_corners;
            int n = symbol->get_location_size();

            for(int i=0;i<n;i++)
            {
                qr_corners.push_back(cv::Point(symbol->get_location_x(i),symbol->get_location_y(i)));
            }

            cv::RotatedRect r = cv::minAreaRect(qr_corners);
            cv::Point2f pts[4];
            r.points(pts);

            for(int i=0;i<4;i++)
            {
                cv::line(cv_ptr->image,pts[i],pts[(i+1)%4],cv::Scalar(0,0,255),3);
            }
        }
    }

    if(preview_param)
    {
        cv::imshow("QrCamera", cv_ptr->image);
        cv::waitKey(5);
    }
}

void Qr_decoder::imageRowCallback(const sensor_msgs::Image::ConstPtr msg)
{
    if(search_flag == 1)
    {
        this->decode(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8));
    }
}
void Qr_decoder::begin_search()
{
    search_flag = 1;
}
bool Qr_decoder::end_search()
{
    if(search_flag == 0)
        return true;

    return false;

}

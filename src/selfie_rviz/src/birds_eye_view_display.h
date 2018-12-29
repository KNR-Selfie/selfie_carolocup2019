#ifndef BIRDS_EYE_VIEW_DISPLAY_H
#define BIRDS_EYE_VIEW_DISPLAY_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <rviz/display.h>

namespace rviz
{
class RosTopicProperty;
class	TfFrameProperty;
class FloatProperty;
}

namespace ros
{
class NodeHandle;
}

namespace selfie_rviz
{

class BirdsEyeViewVisual;

class BirdsEyeViewDisplay: public rviz::Display
{
	Q_OBJECT
public:
	BirdsEyeViewDisplay();
	virtual ~BirdsEyeViewDisplay();
	virtual void setTopic(const QString &topic, const QString &datatype);

protected:
	virtual void onInitialize();
	virtual void reset();

private Q_SLOTS:
	void updateImageTopic();
	void updateReferenceFrame();
	void updateViewArea();

private:
	void imageCallback(const sensor_msgs::Image::ConstPtr& img);

	ros::NodeHandle nh_;
	ros::Subscriber image_subscriber_;

	BirdsEyeViewVisual* visual_;

	rviz::RosTopicProperty* image_topic_property_;
	rviz::TfFrameProperty*	frame_property_;
	rviz::FloatProperty*		min_x_property_;
	rviz::FloatProperty*		max_x_property_;
	rviz::FloatProperty*		min_y_property_;
	rviz::FloatProperty*		max_y_property_;
};

} // end namespace selfie_rviz

#endif // BIRDS_EYE_VIEW_DISPLAY_H

#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/display_context.h>

#include <ros/node_handle.h>

#include "birds_eye_view_visual.h"
#include "birds_eye_view_display.h"

namespace selfie_rviz
{

BirdsEyeViewDisplay::BirdsEyeViewDisplay() : visual_(nullptr)
{
	image_topic_property_ = new rviz::RosTopicProperty("Topic", "",
        "sensor_msgs/Image",
				"",
				this, SLOT(updateImageTopic()));

	frame_property_ = new rviz::TfFrameProperty("Reference Frame", rviz::TfFrameProperty::FIXED_FRAME_STRING,
        "", this, NULL, false,
				SLOT(updateReferenceFrame()));

	min_x_property_  = new rviz::FloatProperty("Min X", -3.0,
        "", this, SLOT(updateViewArea()));

	max_x_property_  = new rviz::FloatProperty("Max X",  3.0,
				"", this, SLOT(updateViewArea()));

	min_y_property_  = new rviz::FloatProperty("Min Y", -3.0,
				"", this, SLOT(updateViewArea()));

	max_y_property_  = new rviz::FloatProperty("Max Y",  3.0,
				"", this, SLOT(updateViewArea()));
}

BirdsEyeViewDisplay::~BirdsEyeViewDisplay()
{
}

void BirdsEyeViewDisplay::setTopic(const QString &topic, const QString &datatype)
{
	image_topic_property_->setString(topic);
}

void BirdsEyeViewDisplay::onInitialize()
{
	frame_property_->setFrameManager(context_->getFrameManager());
}

void BirdsEyeViewDisplay::reset()
{
	ROS_INFO("Reset");
}

void BirdsEyeViewDisplay::updateImageTopic()
{
	std::string topic = image_topic_property_->getTopicStd();

	image_subscriber_ = nh_.subscribe(topic, 10, &BirdsEyeViewDisplay::imageCallback, this);
}

void BirdsEyeViewDisplay::updateReferenceFrame()
{
	reset();
}

void BirdsEyeViewDisplay::updateViewArea()
{
	if (!visual_) return;

	visual_->setViewArea(min_x_property_->getFloat(),
											 max_x_property_->getFloat(),
										 	 min_y_property_->getFloat(),
										 	 max_y_property_->getFloat());
}

void BirdsEyeViewDisplay::imageCallback(const sensor_msgs::Image::ConstPtr& img)
{
	if (!visual_)
	{
		visual_ = new BirdsEyeViewVisual(context_->getSceneManager(), scene_node_);
		updateViewArea();
	}

	visual_->setImage(img);
}

} // end namespace selfie_rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(selfie_rviz::BirdsEyeViewDisplay, rviz::Display)

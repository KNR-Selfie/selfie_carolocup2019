#ifndef BIRDS_EYE_VIEW_VISUAL_H
#define BIRDS_EYE_VIEW_VISUAL_H

#include <OGRE/OgreVector2.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>

#include <rviz/image/ros_image_texture.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>

namespace selfie_rviz
{

class BirdsEyeViewVisual
{
public:
	BirdsEyeViewVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
	virtual ~BirdsEyeViewVisual();

	void setImage(const sensor_msgs::Image::ConstPtr& img);

	void setFramePosition(const Ogre::Vector3& position);
	void setFrameOrientation(const Ogre::Quaternion& orientation);

	void setViewArea(float min_x, float max_x, float min_y, float max_y);
	void setHomography(cv::Mat world2cam);

private:
	float min_x_;
	float max_x_;
	float min_y_;
	float max_y_;

	cv::Mat world2cam_;

	float pixel_size_; // TODO

	Ogre::ManualObject* quad_object_;
	Ogre::MaterialPtr quad_mat_;
	rviz::ROSImageTexture* quad_texture_;

	Ogre::SceneNode* frame_node_;
	Ogre::SceneManager* scene_manager_;

	void createQuad_();
	void updateQuadDimensions_(float min_x, float max_x, float min_y, float max_y);
};

} // end namespace selfie_rviz

#endif // BIRDS_EYE_VIEW_VISUAL_H

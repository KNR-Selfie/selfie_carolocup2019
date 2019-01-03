#include <rviz/uniform_string_stream.h>
#include <opencv2/calib3d.hpp>

#include "birds_eye_view_visual.h"

namespace selfie_rviz
{

BirdsEyeViewVisual::BirdsEyeViewVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
	scene_manager_ = scene_manager;
	frame_node_ = parent_node->createChildSceneNode();

	// TODO
	pixel_size_ = 0.01;

	createQuad_();
}

BirdsEyeViewVisual::~BirdsEyeViewVisual()
{
	scene_manager_->destroySceneNode(frame_node_);
}

void BirdsEyeViewVisual::setImage(const sensor_msgs::Image::ConstPtr& img)
{
	cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img);
	cv::Mat dest;
	cv::warpPerspective(cv_img->image, dest, topview2cam_, topview_size_, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);

	cv_img->image = dest;
	sensor_msgs::Image::Ptr warped_img(new sensor_msgs::Image);
	cv_img->toImageMsg(*warped_img);

	quad_texture_->addMessage(warped_img);
	quad_texture_->update();
}

void BirdsEyeViewVisual::setFramePosition(const Ogre::Vector3& position)
{
	frame_node_->setPosition(position);
}

void BirdsEyeViewVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
	frame_node_->setOrientation(orientation);
}

void BirdsEyeViewVisual::setViewArea(float min_x, float max_x,
																		 float min_y, float max_y)
{
	min_x_ = min_x;
	max_x_ = max_x;
	min_y_ = min_y;
	max_y_ = max_y;

	updateQuadDimensions_();
}

void BirdsEyeViewVisual::setHomography(cv::Mat world2cam)
{
	world2cam_ = world2cam;
	computeTopView_();
}

void BirdsEyeViewVisual::createQuad_()
{
	rviz::UniformStringStream ss;

	ss << "BirdsEyeView Quad" << 1; // TODO
	quad_object_ = scene_manager_->createManualObject(ss.str());
	frame_node_->attachObject(quad_object_);

	ss << " Material";

  quad_mat_ = Ogre::MaterialManager::getSingleton().create(ss.str(), "rviz");
  quad_mat_->setReceiveShadows(false);
  quad_mat_->setCullingMode(Ogre::CULL_CLOCKWISE);

	Ogre::Technique* tech = quad_mat_->getTechnique(0);
  Ogre::Pass* pass = tech->getPass(0);

	quad_texture_ = new rviz::ROSImageTexture();
	pass->createTextureUnitState()->setTextureName(
		quad_texture_->getTexture()->getName()
	);
}

void BirdsEyeViewVisual::updateQuadDimensions_()
{
	quad_object_->clear();
	quad_object_->estimateVertexCount(8);
	quad_object_->begin(quad_mat_->getName(),
											Ogre::RenderOperation::OT_TRIANGLE_LIST);

	Ogre::Vector3 points[4];
	points[0].x = max_x_;
	points[0].y = max_y_;
	points[0].z = 0;

	points[1].x = max_x_;
	points[1].y = min_y_;
	points[1].z = 0;

	points[2].x = min_x_;
	points[2].y = min_y_;
	points[2].z = 0;

	points[3].x = min_x_;
	points[3].y = max_y_;
	points[3].z = 0;

	int triangle_vertex_indices[2][3];
	triangle_vertex_indices[0][0] = 0;
	triangle_vertex_indices[0][1] = 3;
	triangle_vertex_indices[0][2] = 1;
	triangle_vertex_indices[1][0] = 1;
	triangle_vertex_indices[1][1] = 3;
	triangle_vertex_indices[1][2] = 2;

	Ogre::Vector2 tex_coords[4];
	tex_coords[0].x = 0.0;
	tex_coords[0].y = 0.0;
	tex_coords[1].x = 1.0;
	tex_coords[1].y = 0.0;
	tex_coords[2].x = 1.0;
	tex_coords[2].y = 1.0;
	tex_coords[3].x = 0.0;
	tex_coords[3].y = 1.0;

	// iterate over triangles
	for (int i = 0; i < 2; i++) {
		Ogre::Vector3 v[3];
		Ogre::Vector2 tc[3];
		for (int c = 0; c < 3; c++) {
				v[c] = Ogre::Vector3(
						points[triangle_vertex_indices[i][c]].x,
						points[triangle_vertex_indices[i][c]].y,
						points[triangle_vertex_indices[i][c]].z);
				tc[c] = Ogre::Vector2(
						tex_coords[triangle_vertex_indices[i][c]].x,
						tex_coords[triangle_vertex_indices[i][c]].y);
		}

		Ogre::Vector3 normal = (v[1] - v[0]).crossProduct(v[2] - v[0]);
		normal.normalise();

		// front-face
		for (size_t c = 0; c < 3; c++) {
			quad_object_->position(v[c]);
			quad_object_->normal(normal);
			quad_object_->textureCoord(tc[c]);
		}

		// back-face
		for (size_t c = 0; c < 3; c++) {
			quad_object_->position(v[2 - c]);
			quad_object_->normal(-normal);
			quad_object_->textureCoord(tc[2 - c]);
		}
	}

	quad_object_->end();
}

void BirdsEyeViewVisual::computeTopView_()
{
	// Choose top-view image size
	topview_size_ = cv::Size((max_y_ - min_y_) / pixel_size_,
													 (max_x_ - min_x_) / pixel_size_);

	// Choose corner points (in real-world coordinates)
	std::vector<cv::Point2f> coordinates;
	coordinates.emplace_back(min_x_, min_y_);
	coordinates.emplace_back(min_x_, max_y_);
	coordinates.emplace_back(max_x_, min_y_);
	coordinates.emplace_back(max_x_, max_y_);

	std::vector<cv::Point2f> pixels;
	pixels.emplace_back(topview_size_.width, topview_size_.height);
	pixels.emplace_back(0, topview_size_.height);
	pixels.emplace_back(topview_size_.width, 0);
	pixels.emplace_back(0, 0);

	cv::Mat H = cv::findHomography(pixels, coordinates);

	topview2cam_ = world2cam_ * H;
}

} // end namespace selfie_rviz

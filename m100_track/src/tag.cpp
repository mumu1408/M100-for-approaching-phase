#include <tag.h>
#include <stdexcept>

Tag::Tag()
{
	position_camera_frame_ << 0, 0, 0;
	position_drone_frame_ << 0, 0, 0;
	tag_to_camera_euler << 0, 0, 0;
	orientation_camera_frame_ = Eigen::Quaternion<double>(1, 0, 0, 0);
	orientation_drone_frame_ = Eigen::Quaternion<double>(1, 0, 0, 0);
	to_landing_center_translation_ << 0, 0, 0;
	//tag_to_camera_transformation_ = Eigen::Isometry3d::Identity();
	//camera_to_drone_transformation_ = Eigen::Isometry3d::Identity();
	//tag_to_drone_transformation_ = Eigen::Isometry3d::Identity();
	tag_to_camera_transformation << 0, 0, 0, 0,
									0, 0, 0, 0,
									0, 0, 0, 0,
									0, 0, 0, 1;
	camera_to_drone_transformation << 0, 0, 0, 0,
									  0, 0, 0, 0,
									  0, 0, 0, 0,
									  0, 0, 0, 1;
	tag_to_drone_transformation << 0, 0, 0, 0,
								   0, 0, 0, 0,
								   0, 0, 0, 0,
								   0, 0, 0, 1;
	landing_center_position_ << 0, 0, 0;
	found_ = false;
	to_landing_center_translation_set_ = false;
}


void Tag::updateTagState(const geometry_msgs::Pose tag_pose)
{
	position_camera_frame_ = Eigen::Vector3d(tag_pose.position.x, tag_pose.position.y, tag_pose.position.z);
	orientation_camera_frame_ = Eigen::Quaternion<double>(tag_pose.orientation.w, tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z);
	//tag_to_camera_transformation_.rotate(orientation_camera_frame_.toRotationMatrix());
	//tag_to_camera_transformation_.pretranslate(position_camera_frame_);

	tag_to_camera_euler = orientation_camera_frame_.toRotationMatrix().eulerAngles(2, 1, 0);
	std::cout << "tag_to_camera_euler around x axis = " << tag_to_camera_euler[2] << std::endl;
	std::cout << "tag_to_camera_euler around y axis = " << tag_to_camera_euler[1] << std::endl;
	std::cout << "tag_to_camera_euler around z axis = " << tag_to_camera_euler[0] << std::endl;

	tag_to_camera_transformation.block(0,0,3,3) = orientation_camera_frame_.toRotationMatrix();
	tag_to_camera_transformation.block(0,3,3,1) = position_camera_frame_;
	//std::cout << "tag_to_camera_position = \n" << position_camera_frame_ <<std::endl;
	//std::cout << "tag_to_camera_rotation = \n" << orientation_camera_frame_.toRotationMatrix() << std::endl;
	//std::cout << "tag_to_camera_transformation matrix = \n" << tag_to_camera_transformation <<std::endl;
	found_ = true;
}

double Tag::getYaw_incameraframe()
{
	return atan2(2*(orientation_camera_frame_.w() * orientation_camera_frame_.z() + orientation_camera_frame_.x() * orientation_camera_frame_.y()), 1 - 2 * (orientation_camera_frame_.y() * orientation_camera_frame_.y() + orientation_camera_frame_.z() * orientation_camera_frame_.z()));
	// return orientation_drone_frame_.toRotationMatrix().eulerAngles(0, 1, 2)(2);
}


void Tag::setToLandingCenterTranslation(const Eigen::Vector3d trans)
{
	to_landing_center_translation_ = trans;
	to_landing_center_translation_set_ = true;
}

double Tag::getYawError()
{
	return atan2(2*(orientation_drone_frame_.w() * orientation_drone_frame_.z() + orientation_drone_frame_.x() * orientation_drone_frame_.y()), 1 - 2 * (orientation_drone_frame_.y() * orientation_drone_frame_.y() + orientation_drone_frame_.z() * orientation_drone_frame_.z()));
	// return orientation_drone_frame_.toRotationMatrix().eulerAngles(0, 1, 2)(2);
}

void Tag::calculateDroneFramePosition(const Eigen::Matrix3d camera_to_drone_rotation)
{
	position_drone_frame_ = camera_to_drone_rotation * position_camera_frame_;
	//camera_to_drone_transformation_.rotate(camera_to_drone_rotation);
	//camera_to_drone_transformation_.pretranslate(Eigen::Vector3d (0,0,0));
	camera_to_drone_transformation.block(0,0,3,3) = camera_to_drone_rotation;
	camera_to_drone_transformation.block(0,3,3,1) = Eigen::Vector3d (0,0,0);
	//std::cout << "camera_to_drone_rotation = \n" << camera_to_drone_rotation <<std::endl;
	//std::cout << "camera_to_drone_transformation matrix = \n" << camera_to_drone_transformation <<std::endl;
}

void Tag::calculateDroneFrameOrientation(const Eigen::Matrix3d camera_to_drone_rotation)
{
	orientation_drone_frame_ = camera_to_drone_rotation * orientation_camera_frame_.toRotationMatrix();
	//tag_to_drone_transformation_.rotate(orientation_drone_frame_.toRotationMatrix());
	//tag_to_drone_transformation_.pretranslate(position_drone_frame_);
	tag_to_drone_transformation.block(0,0,3,3) = orientation_drone_frame_.toRotationMatrix();
	tag_to_drone_transformation.block(0,3,3,1) = position_drone_frame_;
	//std::cout << "tag_to_drone_position = \n" << position_drone_frame_ << std::endl;
	//std::cout << "tag_to_drone_rotation = \n" << orientation_drone_frame_.toRotationMatrix() << std::endl;
	//std::cout << "tag_to_drone_transformation matrix = \n" << tag_to_drone_transformation <<std::endl;
}

void Tag::setMissing()
{
	found_ = false;
}

void Tag::setFound()
{
	found_ = true;
}
Eigen::Vector3d Tag::getLandingCenterPosition()
{
	Eigen::Vector4d target_to_tag_position;
	Eigen::Vector4d target_to_drone_position;
	target_to_tag_position = Eigen::Vector4d(0,0,1,1);
	if(!to_landing_center_translation_set_)
	{
		throw std::runtime_error("To landing center translation is not set!");
	}
	// orientation_drone_frame_ = camera_to_drone_transformation * orientation_camera_frame_.toRotationMatrix();
	// position_drone_frame_ = camera_to_drone_transformation * position_camera_frame_;
	
	//landing_center_position_ = orientation_drone_frame_.matrix() * to_landing_center_translation_ + position_drone_frame_;
	
	//landing_center_position_ = position_drone_frame_;
	target_to_drone_position = camera_to_drone_transformation * tag_to_camera_transformation * target_to_tag_position;
	landing_center_position_ = target_to_drone_position.head(3);
	return landing_center_position_;
}
Eigen::Quaternion<double> Tag::getLandingCenterOrientation()
{
	// orientation_drone_frame_ = camera_to_drone_transformation * orientation_camera_frame_.toRotationMatrix();
	return orientation_drone_frame_;
}

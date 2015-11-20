#include "Camera.h"

namespace
{
	struct QuaternionPose
	{
		QuaternionPose(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation) : rotation(rotation), translation(translation) { };
		Eigen::Quaterniond rotation;
		Eigen::Vector3d translation;
	};

	// Convert pose matrix (Rt) to parameter vector.
	Eigen::VectorXd toParams(const Eigen::Matrix4d& pose)
	{
		auto quaternion = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
		auto quaternionVector = quaternion.coeffs();
		auto translation = pose.block<3, 1>(0, 3);

		Eigen::VectorXd params(quaternionVector.size() + translation.size());
		params << quaternionVector, translation;

		return params;
	}

	// Extract quaternion and translation vector form Rt pose matrix
	QuaternionPose toQuaternionPose(const Eigen::Matrix4d& pose)
	{
		Eigen::Quaterniond rotation = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
		Eigen::Vector3d translation = pose.block<3, 1>(0, 3);
		return QuaternionPose(rotation, translation);
	}

	Eigen::Matrix4d toRtPose(const Eigen::Quaterniond &rotation, const Eigen::Vector3d translation)
	{
		Eigen::Matrix4d Rt = Eigen::Matrix4d::Identity();
		Rt.block<3, 3>(0, 0) = rotation.toRotationMatrix();
		Rt.block<3, 1>(0, 3) = translation;
		return Rt;
	}

	Eigen::Matrix4d inverse_pose(const Eigen::Matrix4d& pose)
	{
		Eigen::Matrix4d ipose = Eigen::Matrix4d::Identity();
		Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
		Eigen::Vector3d t = pose.block<3, 1>(0, 3);
		ipose.block<3, 3>(0, 0) = R.transpose();
		ipose.block<3, 1>(0, 3) = -R.transpose() * t;
		return ipose;
	}

	Eigen::Matrix4d relative_pose(const Camera& ref_cam, const Camera& relative_cam)
	{
		return relative_cam.pose() * inverse_pose(ref_cam.pose());
	}
}

Camera::Camera(int id, const Eigen::Vector2d &focal, const Eigen::Vector2d &principal_point, const Eigen::VectorXd &disto_coeffs, const Eigen::Matrix3d &rotation, const Eigen::Vector3d translation)
: m_id(id), 
  focal(focal), principal_point(principal_point), disto_coeffs(disto_coeffs),
  rotation(rotation), translation(translation)
{
	this->rotation.normalize();
}

Eigen::Vector2d Camera::project(const Eigen::Vector3d &point3D, const Eigen::Vector2d &focal, const Eigen::Vector2d &principal_point, const Eigen::VectorXd &disto_coeffs, const Eigen::Quaterniond &rotation, const Eigen::Vector3d translation) const
{
	// Camera coordinates
	Eigen::Matrix3d R = rotation.toRotationMatrix();
	Eigen::Vector3d P = R*point3D + translation;

	// Perspective projection
	Eigen::Vector2d p = P.segment(0,2) / P[2];

	// Undistort
	double p_norm = p.norm();
	double r = 1.0 + disto_coeffs[0]*p_norm + disto_coeffs[1]*p_norm*p_norm; //@todo : add third degree
	p = r * p;
	
	// Image coordinates
	return focal.asDiagonal() * p + principal_point;
}

std::vector<Eigen::Vector2d> Camera::project(const Eigen::Vector3d &point3D, const Eigen::Quaterniond &rotation, const Eigen::Vector3d translation) const
{
	return { project(point3D, focal, principal_point, disto_coeffs, rotation, translation) };
}

std::vector<Eigen::Vector2d> Camera::project(const Eigen::Vector3d &point3D) const
{
	return { project(point3D, focal, principal_point, disto_coeffs, rotation, translation) };
}

Eigen::Matrix4d Camera::pose() const
{
	return toRtPose(rotation, translation);
}

Eigen::VectorXd Camera::variableParamsVector() const
{
	Eigen::VectorXd params(variableParamsLength());
	params << rotation.coeffs(), translation;
	return params;
}

void Camera::updateVariableParams(const Eigen::VectorXd &params)
{
	Eigen::Vector4d tmp = params.segment(0, 4);
	rotation = Eigen::Quaterniond(tmp);
	rotation.normalize();
	translation = params.segment(4, 3);
}

unsigned int Camera::variableParamsLength() const
{
	return static_cast<unsigned int>(rotation.coeffs().size() + translation.size());
}

StereoCamera::StereoCamera(std::shared_ptr<Camera> reference_cam, std::shared_ptr<Camera> relative_cam)
	: m_reference_cam(reference_cam),
	m_relative_cam(relative_cam)
{
	m_constraint = relative_pose(*m_reference_cam, *m_relative_cam);

#ifdef _DEBUG
	Eigen::Vector3d pt3d(1.0, 2.0, 3.0);
	
	auto proj_combined = project(pt3d);
	auto proj_ref = m_reference_cam->project(pt3d)[0];
	auto proj_rel = m_relative_cam->project(pt3d)[0];
	assert(proj_combined[0].isApprox(proj_ref));
	assert(proj_combined[1].isApprox(proj_rel));

	auto ref_pose = toQuaternionPose(m_reference_cam->pose());
	auto rotation = ref_pose.rotation;
	auto translation = ref_pose.translation;
	proj_combined = project(pt3d, rotation, translation);
	proj_ref = m_reference_cam->project(pt3d, rotation, translation)[0];
	auto rel_pose = toQuaternionPose(m_relative_cam->pose());
	rotation = rel_pose.rotation;
	translation = rel_pose.translation;
	proj_rel = m_relative_cam->project(pt3d, rotation, translation)[0];
	assert(proj_combined[0].isApprox(proj_ref));
	assert(proj_combined[1].isApprox(proj_rel));
#endif

}

std::vector<Eigen::Vector2d> StereoCamera::project(const Eigen::Vector3d &point3D, const Eigen::Quaterniond &rotation, const Eigen::Vector3d translation) const
{
	auto relRt = toQuaternionPose(m_constraint * toRtPose(rotation, translation));
	return { m_reference_cam->project(point3D, rotation, translation)[0], m_relative_cam->project(point3D, relRt.rotation, relRt.translation)[0] };
}

std::vector<Eigen::Vector2d> StereoCamera::project(const Eigen::Vector3d &point3D) const
{
	auto relRt = toQuaternionPose(m_constraint * m_reference_cam->pose());
	return { m_reference_cam->project(point3D)[0], m_relative_cam->project(point3D, relRt.rotation, relRt.translation)[0] };
}

Eigen::VectorXd StereoCamera::variableParamsVector() const
{
	return m_reference_cam->variableParamsVector();
}

unsigned int StereoCamera::variableParamsLength() const
{
	return m_reference_cam->variableParamsLength();
}

void StereoCamera::updateVariableParams(const Eigen::VectorXd &params)
{
	m_reference_cam->updateVariableParams(params);
	auto pose = m_constraint * m_reference_cam->pose();
	m_relative_cam->updateVariableParams(toParams(pose));
}
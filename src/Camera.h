#ifndef CAMERA_H
#define CAMERA_H

#define EIGEN_DONT_VECTORIZE 
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Eigen>
#include <vector>
#include <memory>

class Camera;
typedef std::vector<std::shared_ptr<Camera>> CameraArray;

struct CameraConstraint
{
	CameraConstraint(int reference, int relative) : reference_cam_index(reference), relative_cam_index(relative) {};
	int reference_cam_index;
	int relative_cam_index;
};

class ICamera
{
public:
	virtual ~ICamera() {};
	virtual std::vector<Eigen::Vector2d> project(const Eigen::Vector3d &point3D, const Eigen::Quaterniond &rotation, const Eigen::Vector3d translation) const = 0;
	virtual std::vector<Eigen::Vector2d> project(const Eigen::Vector3d &point3D) const = 0;

	virtual int measurementCount() const = 0;

	virtual Eigen::VectorXd variableParamsVector() const = 0;
	virtual unsigned int variableParamsLength() const = 0;
	virtual void updateVariableParams(const Eigen::VectorXd &params) = 0;
};

class Camera : public ICamera
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Camera() {};
	Camera(int id, const Eigen::Vector2d &focal, const Eigen::Vector2d &principal_point, const Eigen::VectorXd &disto_coeffs, const Eigen::Matrix3d &rotation, const Eigen::Vector3d translation);
	virtual ~Camera() {};

	Eigen::Vector2d project(const Eigen::Vector3d &point3D, const Eigen::Vector2d &focal, const Eigen::Vector2d &principal_point, const Eigen::VectorXd &disto_coeffs, const Eigen::Quaterniond &rotation, const Eigen::Vector3d translation) const;
	
	Eigen::Matrix4d pose() const;

	// Implement ICamera
	virtual std::vector<Eigen::Vector2d> project(const Eigen::Vector3d &point3D, const Eigen::Quaterniond &rotation, const Eigen::Vector3d translation) const override;
	virtual std::vector<Eigen::Vector2d> project(const Eigen::Vector3d &point3D) const override;

	virtual int measurementCount() const override { return 1; };

	virtual Eigen::VectorXd variableParamsVector() const override;
	virtual unsigned int variableParamsLength() const override;
	virtual void updateVariableParams(const Eigen::VectorXd &params) override;

	friend void write_cams(std::ofstream &file, CameraArray &cam_list);
	friend void write_extended_cams(std::ofstream &file, CameraArray &cam_list);

private:
	int m_id;
	Eigen::Vector2d focal;
	Eigen::Vector2d principal_point;
	Eigen::VectorXd disto_coeffs;
	Eigen::Quaterniond rotation;
	Eigen::Vector3d translation;
};

class StereoCamera : public ICamera
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	StereoCamera() {};
	StereoCamera(std::shared_ptr<Camera> reference_cam, std::shared_ptr<Camera> relative_cam);
	virtual ~StereoCamera() {};

	// Implement ICamera
	virtual std::vector<Eigen::Vector2d> project(const Eigen::Vector3d &point3D, const Eigen::Quaterniond &rotation, const Eigen::Vector3d translation) const override;
	virtual std::vector<Eigen::Vector2d> project(const Eigen::Vector3d &point3D) const override;

	virtual int measurementCount() const override { return 2; };

	virtual Eigen::VectorXd variableParamsVector() const override;
	virtual unsigned int variableParamsLength() const override;
	virtual void updateVariableParams(const Eigen::VectorXd &params) override;

private:
	std::shared_ptr<Camera> m_reference_cam;
	std::shared_ptr<Camera> m_relative_cam;

	Eigen::Matrix4d m_constraint;
};

#endif //CAMERA_H
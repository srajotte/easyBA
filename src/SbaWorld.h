#pragma once

#include <vector>
#include <map>
#include <set>

#include "Camera.h"
#include "Point.h"

class SbaWorld
{
private:
	typedef Eigen::Matrix<float, Eigen::Dynamic, 1> MeasurementVisibilityMask;
	typedef Eigen::Matrix<char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> VisibilityMask;
	typedef std::vector<std::shared_ptr<ICamera>> CameraArray;
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MeasurementMatrix;

	struct CameraIndex
	{
		CameraIndex() : cam(-1), proj(-1) {};
		CameraIndex(int cam, int proj) : cam(cam), proj(proj) {};
		bool operator<(const CameraIndex& other) const { return std::tie(cam, proj) < std::tie(other.cam, other.proj); };

		int cam;
		int proj;
	};

	struct Measurement
	{
		Measurement() {};
		Measurement(const CameraIndex& index, const Eigen::Vector2d& pos) : index(index), position(pos) {};

		CameraIndex index;
		Eigen::Vector2d position;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	typedef std::map<int, CameraIndex> CameraIndexMap;

public:
	typedef std::map<CameraIndex, Measurement> MeasurementMap;

	SbaWorld(std::shared_ptr<::CameraArray> cameras, std::shared_ptr<PointArray> points, const std::vector<CameraConstraint>& constraints);

	const CameraArray& cameras() const;
	MeasurementVisibilityMask measurementVisibilityMask(int cam_idx, int pt_idx) const;
	int measurementDimension() const;

	void run_optimization(bool verbose = true);

private:
	CameraIndexMap build_cameras(std::shared_ptr<::CameraArray> cameras, const std::vector<CameraConstraint>& constraints);
	void build_measurements(const CameraIndexMap& cam_index_map);

	unsigned int measurementsCount() const;
	int computeMeasurementsDimension() const;

	VisibilityMask visibilityMask() const;
	Eigen::VectorXd optimizationParameters() const;
	MeasurementMatrix measurements() const;

	void update_parameters(const Eigen::VectorXd &params);

	std::vector<std::shared_ptr<ICamera>> m_cameras;
	std::shared_ptr<PointArray> m_points;
	// Measurement map (CameraIndex->Measurement) of each 3D point.
	std::vector<MeasurementMap> m_measurements;
	int m_measurement_dim;
};

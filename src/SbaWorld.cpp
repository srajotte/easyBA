#include "SbaWorld.h"

#include <set>
#include <algorithm>
#include <array>
#include <iostream>

#include <sba.h>

namespace
{
	Eigen::VectorXd measurementVector(const SbaWorld::MeasurementMap& measurementMap, int cam_idx, int measurement_dim)
	{
		Eigen::VectorXd vector;
		vector.setZero(measurement_dim);

		for (auto& kv : measurementMap)
		{
			if (kv.first.cam == cam_idx)
			{
				auto& measurement = kv.second;
				const int PROJ_DIM = 2;
				vector.segment<PROJ_DIM>(measurement.index.proj * PROJ_DIM) = measurement.position;
			}
		}

		return vector;
	}

	std::set<int> cameraVisibility(const SbaWorld::MeasurementMap& measurmentMap)
	{
		std::set<int> cameras;
		for (auto& m : measurmentMap)
		{
			cameras.insert(m.second.index.cam);
		}
		return cameras;
	}

	static void sfm_project_point3_mot(int cam_idx, int pt_idx, double *cam_params, double *pt_params,
		double *proj, void *global_params)
	{
		SbaWorld* world = static_cast<SbaWorld*>(global_params);

		// Quaternion parameters order : x,y,z,w
		Eigen::Quaterniond rotation(&cam_params[0]);
		Eigen::Map<const Eigen::Vector3d> translation(&cam_params[4]);
		Eigen::Map<const Eigen::Vector3d> point(pt_params);

		rotation.normalize();
		auto projections = world->cameras().at(cam_idx)->project(point, rotation, translation);
		auto visibility = world->measurementVisibilityMask(cam_idx, pt_idx);
		auto proj_width = world->measurementDimension();

		// Must initialize to zero because not all cameras generate the same number of measurements.
		std::fill_n(proj, proj_width, 0.0);

		for (size_t i = 0; i < projections.size(); ++i)
		{
			float v = visibility[i];
			proj[i * 2] = v * projections[i][0];
			proj[i * 2 + 1] = v * projections[i][1];
		}
	}
}

SbaWorld::SbaWorld(std::shared_ptr<::CameraArray> cameras, std::shared_ptr<PointArray> points, const std::vector<CameraConstraint>& constraints)
	: m_points(points)
{
	auto cam_index_map = build_cameras(cameras, constraints);
	build_measurements(cam_index_map);
	m_measurement_dim = computeMeasurementsDimension();
}

const SbaWorld::CameraArray& SbaWorld::cameras() const
{
	return m_cameras;
}

SbaWorld::MeasurementVisibilityMask SbaWorld::measurementVisibilityMask(int cam_idx, int pt_idx) const
{
	MeasurementVisibilityMask vis;
	vis.setZero(m_cameras[cam_idx]->measurementCount());

	for (auto& kv : m_measurements[pt_idx])
	{
		if (kv.first.cam == cam_idx)
		{
			vis[kv.first.proj] = 1.0f;
		}
	}

	return vis;
}

int SbaWorld::measurementDimension() const
{
	return m_measurement_dim;
}

SbaWorld::VisibilityMask SbaWorld::visibilityMask() const
{
	const size_t n_measurements = measurementsCount();
	const size_t n_cams = m_cameras.size();

	VisibilityMask vis_mask;
	vis_mask.setZero(n_measurements, n_cams);

	const size_t n_points = m_points->size();
	for (size_t i = 0; i < n_points; ++i)
	{
		for (size_t j = 0; j < n_cams; ++j)
		{
			auto& cam_mask = measurementVisibilityMask(int(j), int(i));
			// Visible if at least 1 measurement.
			if (cam_mask.sum() > 0.0)
			{
				vis_mask(i, j) = 1;
			}
		}
	}

	return vis_mask;
}

Eigen::VectorXd SbaWorld::optimizationParameters() const
{
	const size_t n_points = m_points->size();
	const size_t point_dim = 3;
	const size_t n_cams = m_cameras.size();
	const size_t cam_dim = m_cameras.front()->variableParamsLength();

	Eigen::VectorXd params;
	params.resize(n_cams*cam_dim + n_points*point_dim);

	size_t param_idx = 0;

	for (size_t i = 0; i < n_cams; ++i) {
		params.segment(param_idx, cam_dim) = m_cameras[i]->variableParamsVector();
		param_idx += cam_dim;
	}

	for (size_t i = 0; i < n_points; ++i) {
		params.segment(param_idx, point_dim) = (*m_points)[i].position;
		param_idx += point_dim;
	}

	return params;
}

SbaWorld::MeasurementMatrix SbaWorld::measurements() const
{
	const unsigned int n_measurements = measurementsCount();
	const unsigned int measurment_dim = m_measurement_dim;
	MeasurementMatrix measurements;
	measurements = MeasurementMatrix::Zero(n_measurements, measurment_dim);

	size_t current_row = 0;

	for (auto& mm : m_measurements)
	{
		// Measurements follow CameraIndex order.
		std::set<int> cameras;
		for (auto& kv : mm)
		{
			cameras.insert(kv.first.cam);
		}

		for (auto cam : cameras)
		{
			measurements.row(current_row) = measurementVector(mm, cam, measurment_dim);
			++current_row;
		}
	}

	return measurements;
}

void SbaWorld::run_optimization(bool verbose)
{
	auto vis_mask = visibilityMask();
	Eigen::VectorXd parameters = optimizationParameters();
	auto measurements = this->measurements();

	int n_pts = static_cast<int>(m_points->size());
	int n_cams = static_cast<int>(m_cameras.size());
	int n_nonmodif_pts = 0;
	int n_nonmodif_cams = 1; // First camera is always fixed
	char* visibility_mask = vis_mask.data();
	double* params = parameters.data(); // camera parameters followed by point parameters
	int cam_param_dim = m_cameras[0]->variableParamsLength();
	int pt_param_dim = 3;
	double* measures = measurements.data();
	double* covx = NULL;
	int measurement_dim = static_cast<int>(measurements.cols());
	double opts[SBA_OPTSSZ];
	double info[SBA_INFOSZ];
	const int MAX_ITERS = 150;
	const int VERBOSITY = 0;

	opts[0] = 1.0e-3;
	opts[1] = 1.0e-10;
	opts[2] = 1.0e-20;
	opts[3] = 1.0e-12;
	opts[4] = 1.0e-3;

	if (verbose) { std::cout << "Bundle adjustment..."; }
	sba_motstr_levmar(
		n_pts, n_nonmodif_pts,
		n_cams, n_nonmodif_cams,
		visibility_mask,
		params, cam_param_dim, pt_param_dim,
		measures, covx, measurement_dim,
		sfm_project_point3_mot,
		NULL,
		(void *)(this),
		MAX_ITERS, VERBOSITY,
		opts, info);
	
	if (verbose)
	{
		std::cout << "done" << std::endl;

		std::cout << "SBA initial mean reprojection error : " << info[0] / measurements.rows() << std::endl;
		std::cout << "SBA final mean reprojection error : " << info[1] / measurements.rows() << std::endl;
		std::cout << "SBA iterations : " << info[5] << std::endl;
		std::cout << "SBA termination code : " << info[6] << std::endl;
	}

	update_parameters(parameters);
}

void SbaWorld::update_parameters(const Eigen::VectorXd &params)
{
	const size_t n_points = m_points->size();
	const size_t point_dim = 3;
	const size_t n_cams = m_cameras.size();
	const size_t cam_dim = m_cameras.front()->variableParamsLength();

	size_t param_idx = 0;

	for (size_t i = 0; i < n_cams; ++i) {
		m_cameras[i]->updateVariableParams(params.segment(param_idx, cam_dim));
		param_idx += cam_dim;
	}

	for (size_t i = 0; i < n_points; ++i) {
		(*m_points)[i].position = params.segment(param_idx, point_dim);
		param_idx += point_dim;
	}
}

SbaWorld::CameraIndexMap SbaWorld::build_cameras(std::shared_ptr<::CameraArray> cameras, const std::vector<CameraConstraint>& constraints)
{
	std::set<int> remaining_indexes;
	for (int i = 0; i < cameras->size(); ++i)
	{
		remaining_indexes.insert(remaining_indexes.end(), i);
	}

	// Map "cameras" indices to m_cameras indices.
	std::map<int, CameraIndex> cam_index_map;

	// Add stereo (constrained) cameras
	for (auto& constr : constraints)
	{
		m_cameras.push_back(std::make_shared<StereoCamera>((*cameras)[constr.reference_cam_index], (*cameras)[constr.relative_cam_index]));
		remaining_indexes.erase(constr.reference_cam_index);
		remaining_indexes.erase(constr.relative_cam_index);
		int BA_index = static_cast<int>(m_cameras.size()) - 1;
		cam_index_map[constr.reference_cam_index] = CameraIndex(BA_index, 0);
		cam_index_map[constr.relative_cam_index] = CameraIndex(BA_index, 1);
	}

	// Add remaining (mono) cameras
	for (auto i : remaining_indexes)
	{
		m_cameras.push_back((*cameras)[i]);
		int BA_index = static_cast<int>(m_cameras.size()) - 1;
		cam_index_map[i] = CameraIndex(BA_index, 0);
	}

	assert(cameras->size() == cam_index_map.size());

	return cam_index_map;
}

void SbaWorld::build_measurements(const CameraIndexMap& cam_index_map)
{
	for (auto& pt : *m_points)
	{
		MeasurementMap mm;
		for (auto& vis : pt.visibility_list)
		{
			auto index = cam_index_map.at(vis.camera_idx);
			mm.emplace(index, Measurement(index, vis.position));
		}
		m_measurements.push_back(mm);
	}
}

unsigned int SbaWorld::measurementsCount() const
{
	size_t n = 0;
	for (auto& m : m_measurements)
	{
		auto& cameras = cameraVisibility(m);
		n += cameras.size();
	}

	return static_cast<unsigned int>(n);
}

int SbaWorld::computeMeasurementsDimension() const
{
	int maxObservations = 0;
	for (auto& cam : m_cameras)
	{
		maxObservations = std::max(maxObservations, cam->measurementCount());
	}

	const int MEASUREMENT_WIDTH = 2;
	return maxObservations * MEASUREMENT_WIDTH;
}


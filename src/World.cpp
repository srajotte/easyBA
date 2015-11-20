#include "World.h"

#include "SbaWorld.h"

World::World(std::shared_ptr<CameraArray> cameras, std::shared_ptr<PointArray> points, const std::vector<CameraConstraint>& constraints)
	: m_cameras(cameras),
	m_points(points),
	m_constraints(constraints)
{

}

std::shared_ptr<CameraArray> World::cameras() const
{
	return m_cameras;
}

std::shared_ptr<PointArray> World::points() const
{
	return m_points;
}

const std::vector<CameraConstraint>& World::constraints() const
{
	return m_constraints;
}

SbaWorld World::sbaWorld() const
{
	return SbaWorld(m_cameras, m_points, m_constraints);
}

double World::mean_reproj_error() const
{
	auto& point_list = *m_points;
	auto& cam_list = *m_cameras;

	const size_t n_points = point_list.size();
	size_t point_count = 0;
	double error = 0.0;

	for (auto& pt : *m_points)
	{
		for (auto& vis : pt.visibility_list)
		{
			Eigen::Vector3d position = pt.position;
			auto cam = cam_list[vis.camera_idx];
			Eigen::Vector2d measure = vis.position;
			Eigen::Vector2d projection = cam->project(position)[0];

			error += (projection - measure).norm();
			point_count++;
		}
	}

	return error / point_count;
}

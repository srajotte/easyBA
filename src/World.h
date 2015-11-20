#pragma once

#include <vector>

#include "Camera.h"
#include "Point.h"

class SbaWorld;

class World
{
public:
	World(std::shared_ptr<CameraArray> cameras, std::shared_ptr<PointArray> points, const std::vector<CameraConstraint>& constraints);

	std::shared_ptr<CameraArray> cameras() const;
	std::shared_ptr<PointArray> points() const;
	const std::vector<CameraConstraint>& constraints() const;

	SbaWorld sbaWorld() const;

	double mean_reproj_error() const;

private:
	std::shared_ptr<CameraArray> m_cameras;
	std::shared_ptr<PointArray> m_points;
	std::vector<CameraConstraint> m_constraints;
};

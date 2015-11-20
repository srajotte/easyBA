#ifndef	POINT_H
#define POINT_H

#define EIGEN_DONT_VECTORIZE 
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <vector>
#include <Eigen/Eigen>

class Camera;

struct Visibility
{
	unsigned int camera_idx;
	unsigned int keypoint_idx;
	Eigen::Vector2d position;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Point
{
	Eigen::Vector3d position;
	Eigen::Vector3i color; // @TODO : replace with vector of char (color [0,255])
	std::vector<Visibility> visibility_list;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::vector<Point> PointArray;

#endif //POINT_H
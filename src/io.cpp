#include "io.h"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <exception>
#include <sstream>

#include "Camera.h"
#include "Point.h"

struct ObjectCounts
{
	ObjectCounts() : cameras(0), points(0), constraints(0) {};

	unsigned int cameras;
	unsigned int points;
	unsigned int constraints;
};

std::string file_format_string(FileFormat format)
{
	std::string str;

	switch (format) {
	case BUNDLER_FORMAT: str = "BUNDLER_FORMAT"; break;
	case EXTENDED_BUNDLER_FORMAT: str = "EXTENDED_BUNDLER_FORMAT"; break;
	};

	return str;
}

void read_bundler_cams(std::ifstream &file, unsigned int n_cams, CameraArray& cam_list)
{
	cam_list.reserve(n_cams);

	for (unsigned int i = 0; i < n_cams; ++i) {
		double f;
		double k1, k2;
		double r0, r1, r2, r3, r4, r5, r6, r7, r8; // rotation matrix 3x3
		double t0, t1, t2; // translation vector

		file >> f;
		file >> k1 >> k2;
		file >> r0 >> r1 >> r2 >> r3 >> r4 >> r5 >> r6 >> r7 >> r8;
		file >> t0 >> t1 >> t2;

		Eigen::Vector2d focal;
		focal << -f, -f; // Negative focal length due to negative perspective projection in the Bundler camera model.
		Eigen::Vector2d principal_point;
		principal_point << 0.0,  0.0;
		Eigen::Vector2d disto_coeffs;
		disto_coeffs << k1, k2;
		Eigen::Matrix3d R;
		R << r0, r1, r2, r3, r4, r5, r6, r7, r8;
		Eigen::Vector3d t;
		t << t0, t1, t2;

		cam_list.push_back(std::make_shared<Camera>(i, focal, principal_point, disto_coeffs, R, t));
	}
}

void read_extended_bundler_cams(std::ifstream &file, unsigned int n_cams, CameraArray& cam_list)
{
	cam_list.reserve(n_cams);

	for (unsigned int i = 0; i < n_cams; ++i) {
		double fx, fy;
		double cx, cy;
		double k1, k2, k3;
		double r0, r1, r2, r3, r4, r5, r6, r7, r8; // rotation matrix 3x3
		double t0, t1, t2; // translation vector

		file >> fx >> fy;
		file >> cx >> cy;
		file >> k1 >> k2 >> k3;
		file >> r0 >> r1 >> r2 >> r3 >> r4 >> r5 >> r6 >> r7 >> r8;
		file >> t0 >> t1 >> t2;

		Eigen::Vector2d focal;
		focal << fx, fy;
		Eigen::Vector2d principal_point;
		principal_point << cx, cy;
		Eigen::Vector3d disto_coeffs;
		disto_coeffs << k1, k2, k3;
		Eigen::Matrix3d R;
		R << r0, r1, r2, r3, r4, r5, r6, r7, r8;
		Eigen::Vector3d t;
		t << t0, t1, t2;

		cam_list.push_back(std::make_shared<Camera>(i, focal, principal_point, disto_coeffs, R, t));
	}
}

void read_cams(std::ifstream &file, FileFormat format, unsigned int n_cams, CameraArray& cam_list)
{
	switch (format) {
	case BUNDLER_FORMAT: read_bundler_cams(file, n_cams, cam_list); break;
	case EXTENDED_BUNDLER_FORMAT: read_extended_bundler_cams(file, n_cams, cam_list); break;
	}
}

bool visibility_cam_order(const Visibility &a, const Visibility &b)
{
	return a.camera_idx < b.camera_idx;
}

void read_visibility(std::ifstream &file, unsigned int n_vis, std::vector<Visibility> &visibility_list)
{
	visibility_list.reserve(n_vis);

	for (unsigned int i = 0; i < n_vis; ++i) {
		unsigned int cam_idx;
		unsigned int keypoint_idx;
		double x, y;

		file >> cam_idx;
		file >> keypoint_idx;
		file >> x >> y;

		Eigen::Vector2d position;
		position << x, y;

		Visibility visilibity;
		visilibity.camera_idx = cam_idx;
		visilibity.keypoint_idx = keypoint_idx;
		visilibity.position = position;

		visibility_list.push_back(visilibity);
	}

	std::sort(visibility_list.begin(), visibility_list.end(), visibility_cam_order);
}

void read_bundler_points(std::ifstream &file, unsigned int n_points, PointArray& point_list)
{
	point_list.reserve(n_points);

	for (unsigned int i = 0; i < n_points; ++i) {
		double p0, p1, p2; // position
		int r, g, b; // color;
		unsigned int n_vis;

		file >> p0 >> p1 >> p2;
		file >> r >> g >> b;
		file >> n_vis;

		Point point;
		read_visibility(file, n_vis, point.visibility_list);

		point.position << p0, p1, p2;
		point.color << r, g, b;

		point_list.push_back(point);
	}
}

void read_extended_bundler_points(std::ifstream &file, unsigned int n_points, PointArray& point_list)
{
	read_bundler_points(file, n_points, point_list);
}

void read_points(std::ifstream &file, FileFormat format, unsigned int n_points, PointArray& point_list)
{
	switch (format) {
	case BUNDLER_FORMAT: read_bundler_points(file, n_points, point_list); break;
	case EXTENDED_BUNDLER_FORMAT: read_extended_bundler_points(file, n_points, point_list); break;
	}
}

std::vector<std::string> split(const std::string& s, char delim) 
{
	std::vector<std::string> tokens;
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		tokens.push_back(item);
	}
	return tokens;
}

ObjectCounts read_object_counts(std::ifstream& file)
{
	std::string line;
	std::getline(file, line);
	auto tokens = split(line, ' ');

	ObjectCounts counts;
	if (tokens.size() == 2)
	{
		counts.cameras = std::stoi(tokens[0]);
		counts.points = std::stoi(tokens[1]);
	}
	else if (tokens.size() == 3)
	{
		counts.cameras = std::stoi(tokens[0]);
		counts.points = std::stoi(tokens[1]);
		counts.constraints = std::stoi(tokens[2]);
	}
	else
	{
		throw std::runtime_error("Invalid object count line");
	}

	return counts;
}

FileFormat read_file_format(std::ifstream &file)
{
	std::string header;
	std::getline(file, header);
	std::transform(header.begin(), header.end(), header.begin(), tolower);

	FileFormat format;
	if (header.find("extended bundle") != std::string::npos) {
		format = EXTENDED_BUNDLER_FORMAT;
	}
	else if (header.find("bundle") != std::string::npos) {
		format = BUNDLER_FORMAT;
	}
	else {
		throw std::runtime_error("Unkown file format.");
	}

	return format;
}

std::vector<CameraConstraint> read_constraints(std::ifstream &file, unsigned int n_constraints)
{
	std::vector<CameraConstraint> constraints;
	constraints.reserve(n_constraints);

	for (unsigned int i = 0; i < n_constraints; ++i)
	{
		std::string line;
		std::getline(file, line);
		auto tokens = split(line, ' ');
		assert(tokens.size() == 2);
		constraints.push_back(CameraConstraint(std::stoi(tokens[0]), std::stoi(tokens[1])));
	}

	return constraints;
}

BundlerParserOutput read_bundler_out(const std::string &filename)
{
	std::ifstream file;
	file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	file.open(filename);

	FileFormat format = read_file_format(file);
	ObjectCounts counts = read_object_counts(file);

	auto constraints = read_constraints(file, counts.constraints);
	auto cam_list = std::make_shared<CameraArray>();
	read_cams(file, format, counts.cameras, *cam_list);
	auto point_list = std::make_shared<PointArray>();
	read_points(file, format, counts.points, *point_list);

	file.close();

	World world(cam_list, point_list, constraints);

	return BundlerParserOutput(std::move(world), format);
}

void write_cams(std::ofstream &file, CameraArray& cam_list)
{
	file.precision(10);
	file.setf(std::ios_base::scientific);

	size_t n_cams = cam_list.size();
	for (size_t i = 0; i < n_cams; ++i) {
		Camera &cam = *(std::dynamic_pointer_cast<Camera>(cam_list[i]));
		file << cam.focal[0] << " " << cam.disto_coeffs[0] << " " << cam.disto_coeffs[1] << std::endl;
		file << cam.rotation.toRotationMatrix() << std::endl;
		file << cam.translation.transpose() << std::endl;
	}
}

void write_extended_cams(std::ofstream &file, CameraArray& cam_list)
{
	file.precision(10);
	file.setf(std::ios_base::scientific);

	size_t n_cams = cam_list.size();
	for (size_t i = 0; i < n_cams; ++i) {
		Camera &cam = *(std::dynamic_pointer_cast<Camera>(cam_list[i]));
		file << cam.focal[0] << " " << cam.focal[1] << std::endl; 
		file << cam.principal_point[0] << " " << cam.principal_point[1] << std::endl;
		file << cam.disto_coeffs[0] << " " << cam.disto_coeffs[1] << " " << cam.disto_coeffs[2] << std::endl;
		file << cam.rotation.toRotationMatrix() << std::endl;
		file << cam.translation.transpose() << std::endl;
	}
}

void write_visibility_list(std::ofstream &file, Point& pt)
{
	file.precision(4);
	file.unsetf(std::ios_base::scientific);
	file.setf(std::ios_base::fixed, std::ios_base::floatfield);

	size_t n_vis = pt.visibility_list.size();
	file << n_vis << " ";

	for (size_t i = 0; i < n_vis; ++i) {
		Visibility &vis = pt.visibility_list[i];

		file << vis.camera_idx << " " << vis.keypoint_idx << " " << vis.position.transpose() << " ";
	}

	file.unsetf(std::ios_base::fixed);
	file.unsetf(std::ios_base::floatfield);
	file << std::endl;
}

void write_points(std::ofstream &file, std::vector<Point> &point_list)
{
	size_t n_points = point_list.size();
	for (size_t i = 0; i < n_points; ++i) {
		Point &pt = point_list[i];

		file.precision(10);
		file.setf(std::ios_base::scientific);

		file << pt.position.transpose() << std::endl;
		file << pt.color.transpose() << std::endl;

		write_visibility_list(file, pt);
	}
}

void write_classic_bundler(std::ofstream &file, CameraArray& cam_list, PointArray& point_list)
{
	file << "# Bundle file v0.3" << std::endl;
	file << cam_list.size() << " " << point_list.size() << std::endl;

	write_cams(file, cam_list);
	write_points(file, point_list);
}

void write_constraints(std::ofstream &file, const std::vector<CameraConstraint>& constraints)
{
	for (auto& cstr : constraints)
	{
		file << cstr.reference_cam_index << " " << cstr.relative_cam_index << std::endl;
	}
}

void write_extended_bundler(std::ofstream &file, CameraArray& cam_list, PointArray& point_list, const std::vector<CameraConstraint>& constraints)
{
	size_t nConstraints = constraints.size();

	file << "# Extended Bundle file v0.1" << std::endl;
	if (nConstraints > 0)
	{
		file << cam_list.size() << " " << point_list.size() << " " << nConstraints << std::endl;
	}
	else
	{
		file << cam_list.size() << " " << point_list.size() << std::endl;
	}

	write_constraints(file, constraints);
	write_extended_cams(file, cam_list);
	write_points(file, point_list);
}

void write_bundler_out(const std::string &filename, CameraArray& cam_list, PointArray& point_list, const std::vector<CameraConstraint>& constraints, FileFormat format)
{
	std::ofstream file;
	file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	file.open(filename);

	switch (format) {
		case BUNDLER_FORMAT: write_classic_bundler(file, cam_list, point_list); break;
		case EXTENDED_BUNDLER_FORMAT: write_extended_bundler(file, cam_list, point_list, constraints); break;
	}

	file.close();
}
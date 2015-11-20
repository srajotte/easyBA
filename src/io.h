#ifndef IO_H
#define IO_H

#include <string>
#include <vector>

#include "World.h"

#define EIGEN_DONT_VECTORIZE 
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

enum FileFormat { BUNDLER_FORMAT, EXTENDED_BUNDLER_FORMAT };

enum class CameraParameterization
{
	BUNDLER,
	EXTENDED_BUNDLER
};

struct ParsingParameters
{
	ParsingParameters()
		: camera_parameterization(CameraParameterization::BUNDLER),
		has_camera_constraints(false)
	{};

	CameraParameterization camera_parameterization;
	bool has_camera_constraints;
};

struct BundlerParserOutput
{
	BundlerParserOutput(World&& world, FileFormat& format) : world(std::move(world)), fileFormat(format) {};
	World world;
	FileFormat fileFormat;
};

BundlerParserOutput read_bundler_out(const std::string &filename);
void write_bundler_out(const std::string &filename, CameraArray& cam_list, PointArray& point_list, const std::vector<CameraConstraint>& constraints, FileFormat format);
void write_cams(const std::ofstream &file, CameraArray& cam_list);
void write_extended_cams(const std::ofstream &file, CameraArray& cam_list);

/*class FileFormat {};
class BundlerFile : public FileFormat {};
class */


#endif //IO_H
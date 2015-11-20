#include <iostream>
#include <memory>
#include <array>

#include "io.h"
#include "World.h"
#include "SbaWorld.h"

#ifndef NULL
#define NULL 0
#endif

void display_usage()
{
	std::cout << "easyBA.exe input output" << std::endl;
}

int main(int argc, char* argv[])
{
	if (argc != 3) {
		display_usage();
		return -1;
	}

	std::cout << "Loading data...";
	BundlerParserOutput parserOutput = read_bundler_out(std::string(argv[1]));
	std::cout << "done" << std::endl;

	World world = parserOutput.world;
	SbaWorld sbaWorld = world.sbaWorld();

	double avg_reproj_error = world.mean_reproj_error();
	std::cout << "Initial mean reprojection error = " << avg_reproj_error << " pixels" << std::endl;

	sbaWorld.run_optimization();
	
	avg_reproj_error = world.mean_reproj_error();
	std::cout << "Final mean reprojection error : " << avg_reproj_error << " pixels" << std::endl;

	std::cout << "Writing data...";
	write_bundler_out(std::string(argv[2]), *world.cameras(), *world.points(), world.constraints(), parserOutput.fileFormat);
	std::cout << "done" << std::endl;

	return 0;
}
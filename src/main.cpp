#include "Power_diagram_generator.hpp"
#include "Options.hpp"
#include "Time_count.hpp"

#include <fstream>

using namespace PowerDiagramGenerator;

int main(int argc, char** argv) {
	Options& options = getOptions();
	int argn = options.parseOptions(argc, argv);

	if (options.showHelp || argn + 1 != argc) {
		std::cerr << "Usage: " << argv[0] << " [options] points.xyz" << std::endl;
		std::cerr << std::endl;
		options.showOptions(std::cerr);
		return 1;
	}

	TimeCount TC;

	// read point cloud
	std::cout << "read point cloud" << std::endl;
	Generator G;
	TC.start();
	try {
		G.read_pointcloud(argv[argn]);
	}
	catch (const std::runtime_error& e) {
		if (G.read_pointcloud(options.output_path + argv[argn]) == 0) {
			std::cerr << e.what() << std::endl;
			return 0;
		}
	}
	TC.count_time("read point cloud");

	// set mode
	int mode = 0;
	if (options.voronoi) {
		mode |= MODE::VORONOI_DIAGRAM;
	}
	else {
		mode |= MODE::POWER_DIAGRAM;
	}
	if (options.bounding) {
		mode |= MODE::BOUNDING_BOX;
	}
	if (options.exact) {
		mode |= MODE::EXACT_CALCULATION;
	}

	// generate power diagram
	std::cout << "generate power diagram" << std::endl;
	TC.start();
	G.generate_diagram(options.r, mode);
	TC.count_time("generate power diagram");

	// output info file
	std::cout << "output info file" << std::endl;
	TC.start();
	for (int i = 0, i_end = options.outputList.size(); i < i_end; ++i) {
		std::string type = options.outputList[i];

		if (type == "connection" || type == "c") {
			try {
				G.output_cells_connection(options.output_path + "cells_visual_connection.txt", INFO);
			}
			catch (const std::runtime_error& e) {
				std::cerr << e.what() << std::endl;
				return 0;
			}
		}
		else {
			std::cerr << "output type error!" << std::endl;
		}
	}
	TC.count_time("output info file");

	// output visual file
	std::cout << "output visual file" << std::endl;
	TC.start();
	for (int i = 0, i_end = options.visualList.size(); i < i_end; ++i) {
		std::string type = options.visualList[i];

		if (type == "wireframe" || type == "wire" || type == "w") {
			try {
				G.output_cells_wireframe(options.output_path + "cells_visual_wireframe.obj");
			} catch (const std::runtime_error& e) {
				std::cerr << e.what() << std::endl;
				return 0;
			}
		}
		else if (type == "solid" || type == "s") {
			try {
				G.output_cells_solid(options.output_path + "cells_visual_solid.obj");
			}
			catch (const std::runtime_error& e) {
				std::cerr << e.what() << std::endl;
				return 0;
			}
		}
		else if (type == "conection" || type == "c") {
			try {
				G.output_cells_connection(options.output_path + "cells_visual_connection.obj", VISUAL);
			}
			catch (const std::runtime_error& e) {
				std::cerr << e.what() << std::endl;
				return 0;
			}
		}
		else {
			std::cerr << "output type error!" << std::endl;
		}
	}
	TC.count_time("output visual file");

	if (options.timecount) {
		TC.print_time_count();
	}

	return 1;
}
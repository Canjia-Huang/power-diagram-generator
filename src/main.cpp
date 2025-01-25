#include "Power_diagram_generator.hpp"
#include "Options.hpp"
#include "Time_count.hpp"

using namespace PowerDiagramGenerator;

int main(int argc, char** argv)
{
	Options& options = getOptions();
	int argn = options.parseOptions(argc, argv); // parse all options

	// implement all options
	if (options.showHelp || argn + 1 != argc) {
		std::cerr << "Usage: " << argv[0] << " " << "[options] input_file_path" << std::endl;
		std::cerr << std::endl;
		options.showOptions(std::cerr);
		std::cout << "Example: " << argv[0] << " " << "" << std::endl;
		return 0;
	}

	TimeCount TC;
	Generator G;

	// reading point cloud
	std::cout << "Reading point cloud..." << std::endl;
	{
		TC.start();
		if (G.read_pointcloud(argv[argn]) == 0) {
			if (G.read_pointcloud(options.output_path + argv[argn]) == 0) {
				std::cerr << "Error reading point cloud" << std::endl;
				return 0;
			}
		}
		TC.count_time("reading point cloud");
	}

	// setting mode
	int mode = 0;
	{
		if (options.voronoi) mode |= MODE::VORONOI_DIAGRAM;
		if (options.bounding) mode |= MODE::ENTIRE_BOUNDING_BOX;
		if (options.exact) mode |= MODE::EXACT_CALCULATION;
	}

	// generate power diagram
	std::cout << "Generating power diagram..." << std::endl;
	{
		TC.start();
		G.generate_diagram(options.r, mode);
		TC.count_time("generating power diagram");
	}

	// output info file
	if (!options.outputList.empty()) {
		std::cout << "Outputting info file to:" << options.output_path << std::endl;
		{
			TC.start();
			for (size_t i = 0, i_end = options.outputList.size(); i < i_end; ++i) {
				std::ofstream out;

				if (std::string type = options.outputList[i];
					type == "neighbor" || type == "n") {
					out.open(options.output_path + "cells_neighbors.txt");
					if (out.is_open()) {
						G.output_cells_info(out, "neighbor");
					}
					else {
						std::cerr << "Error writing file!" << std::endl;
					}
				}
				else {
					std::cerr << "Error OUTPUT_TYPE!" << std::endl;
				}

				out.close();
			}
			TC.count_time("outputting info file");
		}
	}

	// output visual file
	if (!options.visualList.empty()) {
		std::cout << "Outputting visual file to:" << options.output_path << std::endl;
		{
			TC.start();
			for (size_t i = 0, i_end = options.visualList.size(); i < i_end; ++i) {
				std::ofstream out;

				if (std::string type = options.visualList[i];
					type == "wireframe" || type == "wire" || type == "w") {
					out.open(options.output_path + "cells_visual_wireframe.obj");
					if (out.is_open()) {
						int start_id = 0;
						G.output_cells_wireframe(out);
					}
					else {
						std::cerr << "Error writing file!" << std::endl;
					}
				}
				else if (type == "polyhedron" || type == "p") {
					out.open(options.output_path + "cells_visual_polyhedron.obj");
					if (out.is_open()) {
						int start_id = 0;
						G.output_cells_polyhedron(out, start_id);
					}
					else {
						std::cerr << "Error writing file!" << std::endl;
					}
				}
				else if (type == "connection" || type == "c") {
					out.open(options.output_path +  "cells_visual_connection.obj");
					if (out.is_open()) {
						G.output_cells_info(out, "connection");
					}
					else {
						std::cerr << "Error writing file!" << std::endl;
					}
				}
				else {
					std::cerr << "ERROR VISUAL_TYPE!" << std::endl;
				}

				out.close();
			}
			TC.count_time("output visual file");
		}
	}

	// print time count information
	if (options.timecount) TC.print_time_count();

	return 1;
}
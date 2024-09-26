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

	Generator G;
	try {
		G.read_pointcloud(argv[argn]);
	}
	catch (const std::runtime_error& e) {
		std::cerr << e.what() << std::endl;
		return 0;
	}

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

	G.generate_diagram(options.r, mode);

	if (options.visual) {

	}
}

/*
int main() {
	std::string data_path = "..//..//data//";


	Generator G;
	G.read_pointcloud(data_path + "TEST.xyz");
	G.generate_diagram(0.05, MODE::POWER_DIAGRAM | MODE::BOUNDING_BOX);
	G.output_cells_wireframe("..//..//data//cell_wireframe.obj");
	G.output_cells_solid("..//..//data//cell_solid.obj");

	return 1;
}*/

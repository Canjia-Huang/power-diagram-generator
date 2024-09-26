#include "Power_diagram_generator.hpp"

#include <fstream>

using namespace PowerDiagramGenerator;

int main() {
	std::string data_path = "..//..//data//";


	Generator G;
	G.read_pointcloud(data_path + "TEST.xyz");
	G.generate_diagram(0.05, MODE::POWER_DIAGRAM | MODE::BOUNDING_BOX | MODE::EXACT_CALCULATION);
	G.output_cells_wireframe("..//..//data//cell_wireframe.obj");
	G.output_cells_solid("..//..//data//cell_solid.obj");

	return 1;
	/*
	Cell cell(1, 1, 1);
	cell.init_cube(0.5);

	std::list<Point> points;
	std::list<std::pair<int, int>> edges;
	cell.output_Cell(points, edges);

	std::ofstream out("test.obj");
	for (auto p : points) {
		out << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
	}
	for (auto e : edges) {
		out << "l " << e.first + 1 << " " << e.second + 1 << std::endl;
	}
	out.close();

	return 1;*/
}

#include "Power_diagram_generator.hpp"

#include <fstream>

using namespace PowerDiagramGenerator;

int main() {
	std::string data_path = "..//..//data//";

	std::vector<Point> points;
	read_pointcloud(data_path + "TEST.xyz", points);

	std::vector<std::vector<int>> neighbors;
	const double radis = 0.03;
	get_neighbors(
		points,
		radis,
		neighbors
	);
	std::vector<Cell*> PCs;
	generate_power_diagram(
		points,
		2 * radis,
		neighbors,
		PCs
	);

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

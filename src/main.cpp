#include "Power_diagram_generator.hpp"

#include <fstream>

int main() {

	PowerDiagramGenerator::Cell cell(1, 1, 1);
	cell.init_cube(0.5);

	std::list<PowerDiagramGenerator::Point> points;
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

	return 1;
}

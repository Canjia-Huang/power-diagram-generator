#ifndef POWER_DIAGRAM_GENERATOR_HPP_
#define POWER_DIAGRAM_GENERATOR_HPP_

#ifdef POWER_DIAGRAM_GENERATOR_DEBUG
#define OUTPUT_PATH			"..//..//data//"
// #define OUTPUT_NEIGHBORS	"Neighbors"
#define OUTPUT_POWER_CELLS	"Power_Cells"
#endif

#ifdef POWER_DIAGRAM_GENERATOR_VERBOSE
#	define VERBOSE_ONLY_COUT(x) std::cout << __FUNCTION__ << " " << "\033[33m" << x << "\033[0m" << std::endl // white + yellow color
#else
#	define VERBOSE_ONLY_COUT(x)
#endif

#include "Time_count.hpp"

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <tuple>
#include <list>
#include <map>
#include <unordered_set>
#include <string>

// Nanoflann
#include "nanoflann/nanoflann.hpp"
#include "nanoflann/utils.h"
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud<double> >, PointCloud<double>, 3> my_kd_tree_t;

#define PD_EPS 1e-12
#define PD_MAX 1e12

namespace PowerDiagramGenerator {
	struct pair_hash {
		template <class T1, class T2>
		size_t operator () (std::pair<T1, T2> const& pair) const {
			size_t h1 = std::hash<T1>()(pair.first);
			size_t h2 = std::hash<T2>()(pair.second);
			return h1 ^ h2;
		}
	};

	class Point {
	public:
		Point() {
			x_ = 0;
			y_ = 0;
			z_ = 0;
			w_ = 0;
		}
		~Point() {
			x_ = 0;
			y_ = 0;
			z_ = 0;
			w_ = 0;
		}
		Point(
			double x,  double y, double z) {
			x_ = x;
			y_ = y;
			z_ = z;
			w_ = 0;
		}
		Point(
			double x, double y, double z, double w) {
			x_ = x;
			y_ = y;
			z_ = z;
			w_ = w;
		}
		double& x() { return x_; }
		double& y() { return y_; }
		double& z() { return z_; }
		double& w() { return w_; }
		bool operator <(const Point& p) const {
			if ((x_ - p.x_) < PD_EPS && (x_ - p.x_) > -PD_EPS) {
				if ((y_ - p.y_) < PD_EPS && (y_ - p.y_) > -PD_EPS) {
					return (z_ < p.z_);
				}
				return (y_ < p.y_);
			}
			return (x_ < p.x_);
		}
		bool operator ==(const Point& p) const {
			if ((x_ - p.x_) < PD_EPS && (x_ - p.x_) > -PD_EPS) {
				if ((y_ - p.y_) < PD_EPS && (y_ - p.y_) > -PD_EPS) {
					if ((z_ - p.z_) < PD_EPS && (z_ - p.z_) > -PD_EPS) {
						return true;
					}
				}
			}
			return false;
		}
		Point operator *(double s) {
			Point rp;
			rp.x_ = s * this->x_;
			rp.y_ = s * this->y_;
			rp.z_ = s * this->z_;
			return rp;
		}
		Point operator +(const Point& p) {
			Point rp;
			rp.x_ = this->x_ + p.x_;
			rp.y_ = this->y_ + p.y_;
			rp.z_ = this->z_ + p.z_;
			return rp;
		}
		Point operator -(const Point& p) {
			Point rp;
			rp.x_ = this->x_ - p.x_;
			rp.y_ = this->y_ - p.y_;
			rp.z_ = this->z_ - p.z_;
			return rp;
		}
		void operator =(const Point& p) {
			this->x_ = p.x_;
			this->y_ = p.y_;
			this->z_ = p.z_;
			this->w_ = p.w_;
		}
		double dot(const Point& p) {
			return x_ * p.x_ + y_ * p.y_ + z_ * p.z_;
		}
	protected:
		double x_;
		double y_;
		double z_;
		double w_;
	}; // class Point

	class Plane {
	public:
		Plane() {
			a_ = 0;
			b_ = 0;
			c_ = 0;
			d_ = 0;
		}
		~Plane() {
			a_ = 0;
			b_ = 0;
			c_ = 0;
			d_ = 0;
		}
		Plane(
			double cx, double cy, double cz,
			double nx, double ny, double nz) {
			a_ = nx;
			b_ = ny;
			c_ = nz;
			d_ = -cx * nx - cy * ny - cz * nz;
		}
		Plane(Point cor, Point nor) {
			a_ = nor.x();
			b_ = nor.y();
			c_ = nor.z();
			d_ = -cor.dot(nor);
		}
		double a() const { return a_; }
		double b() const { return b_; }
		double c() const { return c_; }
		double d() const { return d_; }

		double sign_distance(
			double x, double y, double z) const {
			return a_ * x + b_ * y + c_ * z + d_;
		}

		bool is_on_positive_side(
			double x, double y, double z) const {
			return (sign_distance(x, y, z) > 0);
		}
		bool is_on_positive_side(Point& p) const {
			return (sign_distance(p.x(), p.y(), p.z()) > 0);
		}
	protected:
		double a_;
		double b_;
		double c_;
		double d_;
	}; // class Plane

	Point get_tri_planes_cutted_point(
		const Plane& P1,
		const Plane& P2,
		const Plane& P3) {
		double d = P1.a() * (P2.b() * P3.c() - P2.c() * P3.b()) - P1.b() * (P2.a() * P3.c() - P2.c() * P3.a()) + P1.c() * (P2.a() * P3.b() - P2.b() * P3.a());
		double x = -P1.d() * (P2.b() * P3.c() - P2.c() * P3.b()) + P2.d() * (P1.b() * P3.c() - P1.c() * P3.b()) - P3.d() * (P1.b() * P2.c() - P1.c() * P2.b());
		double y = P1.d() * (P2.a() * P3.c() - P2.c() * P3.a()) - P2.d() * (P1.a() * P3.c() - P1.c() * P3.a()) + P3.d() * (P1.a() * P2.c() - P1.c() * P2.a());
		double z = P1.d() * (P2.b() * P3.a() - P2.a() * P3.b()) - P2.d() * (P1.b() * P3.a() - P1.a() * P3.b()) + P3.d() * (P1.b() * P2.a() - P1.a() * P2.b());
		return Point(x / d, y / d, z / d);
	}

	class CutPlane : public Plane {
	public:
		CutPlane(
			double cx, double cy, double cz,
			double nx, double ny, double nz,
			int opposite_id) {
			a_ = nx;
			b_ = ny;
			c_ = nz;
			d_ = -cx * nx - cy * ny - cz * nz;
			opposite_id_ = opposite_id;
		}
		CutPlane(Point cor, Point nor,
			int opposite_id) {
			a_ = nor.x();
			b_ = nor.y();
			c_ = nor.z();
			d_ = -cor.dot(nor);
			opposite_id_ = opposite_id;
		}
		int& opposite_id() { return opposite_id_; }
	protected:
		int opposite_id_;
	}; // class CutPlane : public Plane

	class Cell {
	public:
		Cell() {
			cx_ = 0;
			cy_ = 0;
			cz_ = 0;
		}
		~Cell() {
			cx_ = 0;
			cy_ = 0;
			cz_ = 0;
			(std::vector<CutPlane>()).swap(cutted_planes_);
			(std::vector<std::pair<std::tuple<int, int, int>, Point>>()).swap(cutted_vertices_);
		}
		Cell(
			double cx, double cy, double cz) {
			cx_ = cx;
			cy_ = cy;
			cz_ = cz;
		}
		Cell(Point& p) {
			cx_ = p.x();
			cy_ = p.y();
			cz_ = p.z();
		}
		double x() { return cx_; }
		double y() { return cy_; }
		double z() { return cz_; }

		void init_cube(double r) {
			cutted_planes_.emplace_back(CutPlane(cx_ + r, cy_, cz_, 1, 0, 0, -1));
			cutted_planes_.emplace_back(CutPlane(cx_ - r, cy_, cz_, -1, 0, 0, -1));
			cutted_planes_.emplace_back(CutPlane(cx_, cy_ + r, cz_, 0, 1, 0, -1));
			cutted_planes_.emplace_back(CutPlane(cx_, cy_ - r, cz_, 0, -1, 0, -1));
			cutted_planes_.emplace_back(CutPlane(cx_, cy_, cz_ + r, 0, 0, 1, -1));
			cutted_planes_.emplace_back(CutPlane(cx_, cy_, cz_ - r, 0, 0, -1, -1));

			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(0, 2, 4), Point(cx_ + r, cy_ + r, cz_ + r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(2, 1, 4), Point(cx_ - r, cy_ + r, cz_ + r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(1, 3, 4), Point(cx_ - r, cy_ - r, cz_ + r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(3, 0, 4), Point(cx_ + r, cy_ - r, cz_ + r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(2, 0, 5), Point(cx_ + r, cy_ + r, cz_ - r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(1, 2, 5), Point(cx_ - r, cy_ + r, cz_ - r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(3, 1, 5), Point(cx_ - r, cy_ - r, cz_ - r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(0, 3, 5), Point(cx_ + r, cy_ - r, cz_ - r)));
		}

		bool cut_by_plane(const CutPlane& plane) {
			std::list<std::tuple<int, int, int>> cutted_vertices;
			// find cutted vertices
			for (auto v_it = cutted_vertices_.begin(); v_it != cutted_vertices_.end();) {
				if (plane.is_on_positive_side((*v_it).second)) {
					cutted_vertices.emplace_back((*v_it).first);
					v_it = cutted_vertices_.erase(v_it);
				}
				else {
					v_it++;
				}
			}
			if (cutted_vertices.size() == 0) {
				return false;
			}

			// find cutted edge -> real cutted vertices
			std::unordered_set<int> remain_planes_set;
			for (auto v_it : cutted_vertices_) {
				remain_planes_set.insert(std::get<0>(v_it.first));
				remain_planes_set.insert(std::get<1>(v_it.first));
				remain_planes_set.insert(std::get<2>(v_it.first));
			}
			std::unordered_set<std::pair<int, int>, pair_hash> cutted_edges;
			for (auto it = cutted_vertices.begin(); it != cutted_vertices.end(); it++) {
				std::pair<int, int> edge1 = std::make_pair(std::get<0>(*it), std::get<1>(*it));
				std::pair<int, int> edge2 = std::make_pair(std::get<1>(*it), std::get<2>(*it));
				std::pair<int, int> edge3 = std::make_pair(std::get<2>(*it), std::get<0>(*it));

				auto f_e1 = cutted_edges.find(std::make_pair(std::get<1>(*it), std::get<0>(*it)));
				if (f_e1 == cutted_edges.end()) {
					cutted_edges.insert(edge1);
				}
				else {
					cutted_edges.erase(f_e1);
				}

				auto f_e2 = cutted_edges.find(std::make_pair(std::get<2>(*it), std::get<1>(*it)));
				if (f_e2 == cutted_edges.end()) {
					cutted_edges.insert(edge2);
				}
				else {
					cutted_edges.erase(f_e2);
				}

				auto f_e3 = cutted_edges.find(std::make_pair(std::get<0>(*it), std::get<2>(*it)));
				if (f_e3 == cutted_edges.end()) {
					cutted_edges.insert(edge3);
				}
				else {
					cutted_edges.erase(f_e3);
				}
			}

			// process
			std::list<int> dangling_planes;
			while (cutted_edges.size() > 0) {
				for (auto it = cutted_edges.begin(); it != cutted_edges.end();) {
					if (dangling_planes.size() == 0) {
						dangling_planes.push_back((*it).first);
						dangling_planes.push_back((*it).second);
						it = cutted_edges.erase(it);
						continue;
					}

					if ((*it).first != dangling_planes.back() &&
						(*it).second != dangling_planes.front()) {
						it++;
					}
					else {
						if ((*it).first == dangling_planes.back() && (*it).second == dangling_planes.front()) {

						}
						else {
							if ((*it).first == dangling_planes.back()) {
								dangling_planes.push_back((*it).second);
							}
							if ((*it).second == dangling_planes.front()) {
								dangling_planes.push_front((*it).first);
							}
						}

						it = cutted_edges.erase(it);
					}
				}
			}

			// renew
			if (dangling_planes.size() > 2) {
				cutted_planes_.emplace_back(plane);

				int planes_cnt = 0;
				std::list<int>::iterator prev_it = dangling_planes.end();
				prev_it--;
				for (std::list<int>::iterator it = dangling_planes.begin(); it != dangling_planes.end(); ++it) {
					Plane* plane1 = &(cutted_planes_[(*prev_it)]);
					Plane* plane2 = &(cutted_planes_[(*it)]);

					Point cutted_point = get_tri_planes_cutted_point((*plane1), (*plane2), plane);

					cutted_vertices_.emplace_back(
						std::make_pair(
							std::make_tuple((*prev_it), (*it), cutted_planes_.size() - 1),
							cutted_point));

					prev_it = it;
				}
			}

			return true;
		}

		void output(
			std::vector<Point>& points,
			std::vector<std::pair<int, int>>& edges) const {
			std::map<std::pair<int, int>, std::vector<int>> plane_pair_and_vertices;
			for (int i = 0, i_end = cutted_vertices_.size(); i < i_end; ++i) {
				int a = std::get<0>(cutted_vertices_[i].first);
				int b = std::get<1>(cutted_vertices_[i].first);
				int c = std::get<2>(cutted_vertices_[i].first);
				std::pair<int, int> plane_pair1 = std::make_pair(std::min(a, b), std::max(a, b));
				std::pair<int, int> plane_pair2 = std::make_pair(std::min(b, c), std::max(b, c));
				std::pair<int, int> plane_pair3 = std::make_pair(std::min(c, a), std::max(c, a));

				plane_pair_and_vertices[plane_pair1].push_back(i);
				plane_pair_and_vertices[plane_pair2].push_back(i);
				plane_pair_and_vertices[plane_pair3].push_back(i);
			}

			points.clear();
			for (int i = 0, i_end = cutted_vertices_.size(); i < i_end; ++i) {
				points.emplace_back(cutted_vertices_[i].second);
			}
			edges.clear();
			for (auto pv : plane_pair_and_vertices) {
				if (pv.second.size() == 2) {
					edges.emplace_back(std::make_pair(pv.second[0], pv.second[1]));
				}
			}
		}
	protected:
		double cx_;
		double cy_;
		double cz_;
		std::vector<CutPlane> cutted_planes_;
		std::vector<std::pair<std::tuple<int, int, int>, Point>> cutted_vertices_;
	}; // class Cell

	// other functions
	int read_pointcloud(
		std::string file_path,
		std::vector<Point>& points
	) {
		VERBOSE_ONLY_COUT("");
		points.clear();
		std::string back = file_path.substr(file_path.length() - 3, file_path.length());

		std::ifstream in(file_path);
		if (!in.good()) {
			throw "INPUT_FILE_PATH_INVALID";
			return 0;
		}
		if (back == "xyz") { // .xyz
			std::string sline;
			while (std::getline(in, sline)) {
				std::istringstream ins(sline);
				Point p;
				ins >> p.x() >> p.y() >> p.z() >> p.w();
				points.push_back(p);
			}
		}
		else if (back == "obj") { // .obj without weights
			std::string sline, s0;
			std::string vertex_char = "v";
			std::string face_char = "f";
			while (std::getline(in, sline)) {
				std::istringstream ins(sline);
				ins >> s0;

				if (s0 == vertex_char) {
					Point p;
					ins >> p.x() >> p.y() >> p.z();
					points.push_back(p);
				}
			}
		}
		else if (back == "off") { // .off without weights
			int vertex_num;
			int face_num;
			int edge_num;
			int s;
			if (!in.good()) {
				return 0;
			}
			do {
				in.get();
			} while (in.get() != '\n');

			in >> vertex_num >> face_num >> edge_num;
			for (int i = 0; i < vertex_num; i++) {
				Point p;
				in >> p.x() >> p.y() >> p.z();
				points.push_back(p);
			}
		}
		else {
			throw "INPUT_FILE_TYPE_INVALID";
			return 0;
		}
		in.close();

		return 1;
	}

	int get_neighbors(
		std::vector<Point>& points,
		const double radis,
		std::vector<std::vector<int>>& neighbors
	) {
		VERBOSE_ONLY_COUT("");
		int points_nb = points.size();

		// build kd-tree cloud
		PointCloud<double> cloud;
		cloud.pts.resize(points_nb);
		for (int i = 0; i < points_nb; ++i) {
			cloud.pts[i].x = points[i].x();
			cloud.pts[i].y = points[i].y();
			cloud.pts[i].z = points[i].z();
		}
		my_kd_tree_t index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));

		// get neighbors
		neighbors.reserve(points_nb);
		for (int i = 0; i < points_nb; ++i) {
			double query[3] = { points[i].x(), points[i].y(), points[i].z() };

			std::vector<std::pair<uint32_t, double>> ret_matches;
			nanoflann::SearchParams params;
			const double search_radius = static_cast<double>(radis * radis);
			const size_t nMatches = index.radiusSearch(&query[0], search_radius, ret_matches, params);

			for (int j = 0; j < nMatches; ++j) {
				int nei_j = ret_matches[j].first;
				if (i != nei_j) {
					neighbors[i].push_back(nei_j);
				}
			}
		}

#ifdef OUTPUT_NEIGHBORS
		{
			VERBOSE_ONLY_COUT("output neighbors");
			std::ofstream out(std::string(OUTPUT_PATH) + std::string(OUTPUT_NEIGHBORS) + ".obj");
			for (int i = 0; i < points_nb; ++i) {
				out << "v" << " " << points[i].x() << " " << points[i].y() << " " << points[i].z() << std::endl;
			}
			std::map<std::pair<int, int>, bool> edge_map;
			for (int i = 0; i < points_nb; ++i) {
				for (int j = 0, j_end = neighbors[i].size(); j < j_end; ++j) {
					std::pair<int, int> edge(std::min(i, neighbors[i][j]), std::max(i, neighbors[i][j]));
					if (edge_map.find(edge) == edge_map.end()) {
						out << "l" << " " << edge.first + 1 << " " << edge.second + 1 << std::endl;
						edge_map[edge] = true;
					}
				}
			}
			out.close();
		}
#endif

		return 1;
	}

	int generate_power_diagram(
		std::vector<Point>& points,
		const double radis,
		const std::vector<std::vector<int>>& neighbors,
		std::vector<Cell*>& PCs
	) {
		VERBOSE_ONLY_COUT("");
		int points_nb = points.size();
		PCs = std::vector<Cell*>(points_nb);

// #pragma omp parallel for
		for (int vi = 0; vi < points_nb; ++vi) {
			if (vi > 30) break;
			if (vi % 1 == 0) VERBOSE_ONLY_COUT("process:" << " " << vi);

			Cell* PC = new Cell(points[vi]);
			PC->init_cube(radis);

			for (int j = 0, j_end = neighbors[vi].size(); j < j_end; ++j) {
				int ni = neighbors[vi][j];

				double sq_dis = 
					(points[vi].x() - points[ni].x()) * (points[vi].x() - points[ni].x()) + 
					(points[vi].y() - points[ni].y()) * (points[vi].y() - points[ni].y()) +
					(points[vi].z() - points[ni].z()) * (points[vi].z() - points[ni].z());
				if (sq_dis < PD_EPS) continue;

				double lambda = 0.5 + 0.5 * (points[vi].w() * points[vi].w() - points[ni].w() * points[ni].w()) / sq_dis;
				Point mid_point = (points[vi] * lambda) + (points[ni] * (1 - lambda));
				Point dir = points[ni] - points[vi];
				/*VERBOSE_ONLY_COUT(points[vi].x() << " " << points[vi].y() << " " << points[vi].z());
				VERBOSE_ONLY_COUT(points[ni].x() << " " << points[ni].y() << " " << points[ni].z());
				VERBOSE_ONLY_COUT(lambda);
				VERBOSE_ONLY_COUT(mid_point.x() << " " << mid_point.y() << " " << mid_point.z());
				VERBOSE_ONLY_COUT(dir.x() << " " << dir.y() << " " << dir.z());*/
				PC->cut_by_plane(CutPlane(
					mid_point,
					dir,
					ni));
			}

			PCs[vi] = PC;
		}

		std::vector<Point> ppoints;
		std::vector<std::pair<int, int>> pedges;
		PCs[0]->output(ppoints, pedges);

		std::ofstream out(std::string(OUTPUT_PATH) + "test.obj");
		for (auto p : ppoints) {
			out << "v " << p.x() << " " << p.y() << " " << p.z() << std::endl;
		}
		for (auto e : pedges) {
			out << "l " << e.first + 1 << " " << e.second + 1 << std::endl;
		}
		out.close();

#ifdef OUTPUT_POWER_CELLS
		{
			VERBOSE_ONLY_COUT("output power cells");
			std::ofstream out(std::string(OUTPUT_PATH) + std::string(OUTPUT_POWER_CELLS) + ".obj");
			std::map<Point, int> point_map;
			int p_cnt = 0;
			for (int vi = 0, vi_end = PCs.size(); vi < vi_end; ++vi) {
				Cell* PC = PCs[vi];

				if (PC != NULL) {
					std::vector<Point> cell_points;
					std::vector<std::pair<int, int>> cell_edges;
					PC->output(cell_points, cell_edges);

					for (auto cp : cell_points) {
						if (point_map.find(cp) == point_map.end()) {
							out << "v" << " " << cp.x() << " " << cp.y() << " " << cp.z() << " " << "255 255 255" << std::endl;
							point_map[cp] = p_cnt++;
						}
					}

					for (auto ce : cell_edges) {
						Point* P1 = &(cell_points[ce.first]);
						Point* P2 = &(cell_points[ce.second]);
						out << "l" << " " << point_map[*P1] + 1 << " " << point_map[*P2] + 1 << std::endl;
					}

					out << "v" << " " << PC->x() << " " << PC->y() << " " << PC->z() << " " << "255 0 0" << std::endl;
					p_cnt++;
				}
			}
			out.close();
		}
#endif

		return 1;
	}

} // namespace PowerDiagramGenerator

#endif

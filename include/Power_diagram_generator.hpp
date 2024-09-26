#ifndef POWER_DIAGRAM_GENERATOR_HPP_
#define POWER_DIAGRAM_GENERATOR_HPP_

#ifdef POWER_DIAGRAM_GENERATOR_DEBUG
#define OUTPUT_PATH			"..//..//data//"
// #define OUTPUT_NEIGHBORS			"DEBUG_Neighbors"
// #define OUTPUT_CELLS_WIREFRAME	"DEBUG_Cells_Wireframe"
// #define OUTPUT_CELLS_SOLID		"DEBUG_Cells_Solid"
#endif

#ifdef POWER_DIAGRAM_GENERATOR_VERBOSE
// #	define VERBOSE_ONLY_COUT(x) std::cout << __FUNCTION__ << " " << "\033[33m" << x << "\033[0m" << std::endl // white + yellow color
#	define VERBOSE_ONLY_COUT(x) std::cout << __FUNCTION__ << " " << x  << std::endl // white + yellow color
#else
#	define VERBOSE_ONLY_COUT(x)
#endif

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <tuple>
#include <list>
#include <map>
#include <set>
#include <unordered_set>
#include <string>

// Nanoflann
#include "nanoflann/nanoflann.hpp"
#include "nanoflann/utils.h"
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud<double> >, PointCloud<double>, 3> my_kd_tree_t;

#define PD_EPS 1e-12
#define PD_MAX 1e12
#define PD_SQRT_3 1.7320508075689

namespace PowerDiagramGenerator {
	struct pair_hash {
		template <class T1, class T2>
		size_t operator () (std::pair<T1, T2> const& pair) const {
			size_t h1 = std::hash<T1>()(pair.first);
			size_t h2 = std::hash<T2>()(pair.second);
			return h1 ^ h2;
		}
	};
	enum MODE {
		POWER_DIAGRAM		= 0,
		VORONOI_DIAGRAM		= 1,
		BOUNDING_BOX		= 2,
		EXACT_CALCULATION	= 4
	};
	enum OUTPUT_TYPE {
		VISUAL,
		INFO
	};

	class Point {
	public:
		Point() {
			cor_[0] = 0;
			cor_[1] = 0;
			cor_[2] = 0;
			w_ = 0;
		}
		~Point() {
			cor_[0] = 0;
			cor_[1] = 0;
			cor_[2] = 0;
			w_ = 0;
		}
		Point(
			double x,  double y, double z) {
			cor_[0] = x;
			cor_[1] = y;
			cor_[2] = z;
			w_ = 0;
		}
		Point(
			double x, double y, double z, double w) {
			cor_[0] = x;
			cor_[1] = y;
			cor_[2] = z;
			w_ = w;
		}
		double& x() { return cor_[0]; }
		double& y() { return cor_[1]; }
		double& z() { return cor_[2]; }
		double* cor() { return cor_; }
		double& w() { return w_; }
		bool operator <(const Point& p) const {
			if ((cor_[0] - p.cor_[0]) < PD_EPS && (cor_[0] - p.cor_[0]) > -PD_EPS) {
				if ((cor_[1] - p.cor_[1]) < PD_EPS && (cor_[1] - p.cor_[1]) > -PD_EPS) {
					return (cor_[2] < p.cor_[2]);
				}
				return (cor_[1] < p.cor_[1]);
			}
			return (cor_[0] < p.cor_[0]);
		}
		bool operator ==(const Point& p) const {
			if ((cor_[0] - p.cor_[0]) < PD_EPS && (cor_[0] - p.cor_[0]) > -PD_EPS) {
				if ((cor_[1] - p.cor_[1]) < PD_EPS && (cor_[1] - p.cor_[1]) > -PD_EPS) {
					if ((cor_[2] - p.cor_[2]) < PD_EPS && (cor_[2] - p.cor_[2]) > -PD_EPS) {
						return true;
					}
				}
			}
			return false;
		}
		Point operator *(double s) {
			Point rp;
			rp.cor_[0] = s * this->cor_[0];
			rp.cor_[1] = s * this->cor_[1];
			rp.cor_[2] = s * this->cor_[2];
			return rp;
		}
		Point operator +(const Point& p) {
			Point rp;
			rp.cor_[0] = this->cor_[0] + p.cor_[0];
			rp.cor_[1] = this->cor_[1] + p.cor_[1];
			rp.cor_[2] = this->cor_[2] + p.cor_[2];
			return rp;
		}
		Point operator -(const Point& p) {
			Point rp;
			rp.cor_[0] = this->cor_[0] - p.cor_[0];
			rp.cor_[1] = this->cor_[1] - p.cor_[1];
			rp.cor_[2] = this->cor_[2] - p.cor_[2];
			return rp;
		}
		void operator =(const Point& p) {
			this->cor_[0] = p.cor_[0];
			this->cor_[1] = p.cor_[1];
			this->cor_[2] = p.cor_[2];
			this->w_ = p.w_;
		}
		double dot(const Point& p) {
			return cor_[0] * p.cor_[0] + cor_[1] * p.cor_[1] + cor_[2] * p.cor_[2];
		}
	protected:
		double cor_[3];
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
			return (sign_distance(x, y, z) > PD_EPS);
		}
		bool is_on_positive_side(Point& p) const {
			return (sign_distance(p.x(), p.y(), p.z()) > PD_EPS);
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
		std::vector<CutPlane>& cutted_planes() { return cutted_planes_; }
		std::vector<std::pair<std::tuple<int, int, int>, Point>>& cutted_vertices() { return cutted_vertices_; }

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

		void init_box(
			double min_x, double max_x,
			double min_y, double max_y,
			double min_z, double max_z) {
			double center_x = 0.5 * (min_x + max_x);
			double center_y = 0.5 * (min_y + max_y);
			double center_z = 0.5 * (min_z + max_z);

			cutted_planes_.emplace_back(CutPlane(max_x, center_y, center_z, 1, 0, 0, -1));
			cutted_planes_.emplace_back(CutPlane(min_x, center_y, center_z, -1, 0, 0, -1));
			cutted_planes_.emplace_back(CutPlane(center_x, max_y, center_z, 0, 1, 0, -1));
			cutted_planes_.emplace_back(CutPlane(center_x, min_y, center_z, 0, -1, 0, -1));
			cutted_planes_.emplace_back(CutPlane(center_x, center_y, max_z, 0, 0, 1, -1));
			cutted_planes_.emplace_back(CutPlane(center_x, center_y, min_z, 0, 0, -1, -1));

			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(0, 2, 4), Point(max_x, max_y, max_z)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(2, 1, 4), Point(min_x, max_y, max_z)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(1, 3, 4), Point(min_x, min_y, max_z)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(3, 0, 4), Point(max_x, min_y, max_z)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(2, 0, 5), Point(max_x, max_y, min_z)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(1, 2, 5), Point(min_x, max_y, min_z)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(3, 1, 5), Point(min_x, min_y, min_z)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(0, 3, 5), Point(max_x, min_y, min_z)));
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
					if (isnan(cutted_point.x()) || isnan(cutted_point.y()) || isnan(cutted_point.z()) ||
						isinf(cutted_point.x()) || isinf(cutted_point.y()) || isinf(cutted_point.z())) {

					}
					else {
						cutted_vertices_.emplace_back(
							std::make_pair(
								std::make_tuple((*prev_it), (*it), cutted_planes_.size() - 1),
								cutted_point));
					}
					prev_it = it;
				}
			}

			return true;
		}

		void output_wireframe(
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

		void output_solid(
			std::vector<Point>& points,
			std::vector<std::vector<int>>& faces
		) {
			std::map<std::pair<int, int>, int> edge_to_vertex_map;
			std::vector<std::set<int>> neighbor_planes(cutted_planes_.size());
			for (auto cv : cutted_vertices_) {
				int P1 = std::get<0>(cv.first);
				int P2 = std::get<1>(cv.first);
				int P3 = std::get<2>(cv.first);

				edge_to_vertex_map[std::make_pair(P1, P2)] = points.size();
				edge_to_vertex_map[std::make_pair(P2, P3)] = points.size();
				edge_to_vertex_map[std::make_pair(P3, P1)] = points.size();

				neighbor_planes[P1].insert(P2);
				neighbor_planes[P1].insert(P3);
				neighbor_planes[P2].insert(P1);
				neighbor_planes[P2].insert(P3);
				neighbor_planes[P3].insert(P1);
				neighbor_planes[P3].insert(P2);

				points.push_back(cv.second);
			}

			// get each face's edges
			faces = std::vector<std::vector<int>>(cutted_planes_.size());
			for (int i = 0, i_end = cutted_planes_.size(); i < i_end; ++i) {
				std::list<std::pair<int, int>> edges;
				for (auto f : neighbor_planes[i]) {
					int V1 = edge_to_vertex_map[std::make_pair(f, i)];
					int V2 = edge_to_vertex_map[std::make_pair(i, f)];
					edges.push_back(std::make_pair(V2, V1)); // from - to
				}
				if (edges.size() == 0) continue; // this cut plane is not used now

				// sort edges
				std::list<std::pair<int, int>> ordered_edges;
				ordered_edges.push_back(edges.back());
				edges.pop_back();
				while (edges.size() > 0) {
					for (auto ei = edges.begin(); ei != edges.end();) {
						if ((*ei).second == ordered_edges.front().first) {
							ordered_edges.push_front(*ei);
							ei = edges.erase(ei);
						}
						else if ((*ei).first == ordered_edges.back().second) {
							ordered_edges.push_back(*ei);
							ei = edges.erase(ei);
						}
						else {
							ei++;
						}
					}
				}

				// covert to face
				for (auto oe : ordered_edges) {
					faces[i].push_back(oe.first);
				}
			}

			// renew faces
			for (auto fi = faces.begin(); fi != faces.end();) {
				if ((*fi).size() == 0) {
					fi = faces.erase(fi);
				}
				else {
					fi++;
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

	class Generator {
	public:
		Generator() {};
		~Generator() {
			(std::vector<Point>()).swap(points_);
			(std::vector<Cell*>()).swap(cells_);
		}
		Cell& cell(const int i) {
			return *(cells_[i]);
		}

		int read_pointcloud(
			std::string file_path
		) {
			VERBOSE_ONLY_COUT("");
			(std::vector<Point>()).swap(points_);
			std::string back = file_path.substr(file_path.length() - 3);

			std::ifstream in(file_path);
			if (!in.good()) {
				throw std::runtime_error("read input file error!");
				return 0;
			}
			if (back == "xyz") { // .xyz
				std::string sline;
				while (std::getline(in, sline)) {
					std::istringstream ins(sline);
					Point p;
					ins >> p.x() >> p.y() >> p.z() >> p.w();
					points_.push_back(p);
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
						points_.push_back(p);
					}
				}
			}
			else if (back == "off") { // .off without weights
				int vertex_num;
				int face_num;
				int edge_num;
				int s;
				if (!in.good()) {
					throw std::runtime_error("read input file error!");
					return 0;
				}
				do {
					in.get();
				} while (in.get() != '\n');

				in >> vertex_num >> face_num >> edge_num;
				for (int i = 0; i < vertex_num; i++) {
					Point p;
					in >> p.x() >> p.y() >> p.z();
					points_.push_back(p);
				}
			}
			else {
				throw std::runtime_error("input file type invalid!");
				return 0;
			}
			in.close();

			return 1;
		}

		int get_neighbors(
			const double sq_radis,
			std::vector<std::vector<int>>& neighbors
		) { // disuse
			VERBOSE_ONLY_COUT("");
			int points_nb = points_.size();

			// build kd-tree cloud
			PointCloud<double> cloud;
			cloud.pts.resize(points_nb);
			for (int i = 0; i < points_nb; ++i) {
				cloud.pts[i].x = points_[i].x();
				cloud.pts[i].y = points_[i].y();
				cloud.pts[i].z = points_[i].z();
			}
			my_kd_tree_t index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));

			// get neighbors
			neighbors.clear();
			neighbors.reserve(points_nb);
			for (int i = 0; i < points_nb; ++i) {
				double query[3] = { points_[i].x(), points_[i].y(), points_[i].z() };

				std::vector<std::pair<uint32_t, double>> ret_matches;
				nanoflann::SearchParams params;
				const double search_radius = static_cast<double>(sq_radis);
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

		int generate_diagram(
			const double radis,
			int mode
		) {
			VERBOSE_ONLY_COUT("");
			int points_nb = points_.size();
			cells_ = std::vector<Cell*>(points_nb);
			nanoflann::SearchParams params;
			if (radis < PD_EPS) {
				mode |= BOUNDING_BOX;
				mode |= EXACT_CALCULATION;
			}

			// get bounding box
			double* box = new double[6];
			if (mode & BOUNDING_BOX) {
				box = get_bounding_box(radis);
			}

			// get maximum and minimum weights
			double sq_search_radis = 0;
			if (mode & EXACT_CALCULATION) {
				sq_search_radis = PD_MAX;
			}
			else {
				double max_w = -PD_MAX;
				double min_w = PD_MAX;
				for (int i = 0; i < points_nb; ++i) {
					max_w = std::max(max_w, points_[i].w());
					min_w = std::min(min_w, points_[i].w());
				}
				sq_search_radis = radis * radis - max_w * max_w + min_w * min_w;
				if (sq_search_radis < PD_EPS) {
					throw std::runtime_error("radius or weights setting invalid");
					return 0;
				}
				sq_search_radis = radis + sqrt(sq_search_radis);
				sq_search_radis *= sq_search_radis;
			}
			const double search_radius = static_cast<double>(sq_search_radis);
			VERBOSE_ONLY_COUT("sq_search_radis:" << " " << sq_search_radis);

			// build kd-tree cloud
			PointCloud<double> cloud;
			cloud.pts.resize(points_nb);
			for (int i = 0; i < points_nb; ++i) {
				cloud.pts[i].x = points_[i].x();
				cloud.pts[i].y = points_[i].y();
				cloud.pts[i].z = points_[i].z();
			}
			my_kd_tree_t index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));

#pragma omp parallel for
			for (int vi = 0; vi < points_nb; ++vi) {
				if (vi % 1000 == 0) VERBOSE_ONLY_COUT("process:" << " " << vi);

				Cell* PC = new Cell(points_[vi]);
				if (mode & BOUNDING_BOX) {
					PC->init_box(box[0], box[1], box[2], box[3], box[4], box[5]);
				}
				else {
					PC->init_cube(radis);
				}

				// get neighbors
				std::vector<std::pair<uint32_t, double>> ret_matches;
				const size_t nMatches = index.radiusSearch(points_[vi].cor(), search_radius, ret_matches, params);

				for (int j = 0; j < nMatches; ++j) {
					int ni = ret_matches[j].first;
					if (vi == ni) continue;

					double sq_dis =
						(points_[vi].x() - points_[ni].x()) * (points_[vi].x() - points_[ni].x()) +
						(points_[vi].y() - points_[ni].y()) * (points_[vi].y() - points_[ni].y()) +
						(points_[vi].z() - points_[ni].z()) * (points_[vi].z() - points_[ni].z());
					if (sq_dis < PD_EPS) continue;

					double lambda;
					if (mode & VORONOI_DIAGRAM) {
						lambda = 0.5;
					}
					else{
						lambda = 0.5 + 0.5 * (points_[vi].w() * points_[vi].w() - points_[ni].w() * points_[ni].w()) / sq_dis;
					}
					Point mid_point = (points_[vi] * lambda) + (points_[ni] * (1 - lambda));
					Point dir = points_[ni] - points_[vi];
					if (PC->cut_by_plane(CutPlane(
						mid_point,
						dir,
						ni)
					) == false) {
						if (mode & VORONOI_DIAGRAM) break; // only for voronoi
					}
				}

				cells_[vi] = PC;
			}

			return 1;
		}

		int output_cell_wireframe(
			std::string file_path,
			Cell& cell
		) {
			VERBOSE_ONLY_COUT("");
			std::ofstream out;
			out.open(file_path);
			if (!out) {
				throw std::runtime_error("write file error!");
				return 0;
			}

			std::vector<Point> cell_points;
			std::vector<std::pair<int, int>> cell_edges;
			cell.output_wireframe(cell_points, cell_edges);

			for (auto cp : cell_points) {
				out << "v" << " " << cp.x() << " " << cp.y() << " " << cp.z() << " " << "255 255 255" << std::endl;
			}
			for (auto ce : cell_edges) {
				out << "l" << " " << ce.first + 1 << " " << ce.second + 1 << std::endl;
			}
			out << "v" << " " << cell.x() << " " << cell.y() << " " << cell.z() << " " << "255 0 0" << std::endl;
			out.close();

			return 1;
		}

		int output_cells_wireframe(
			std::string file_path
		) {
			VERBOSE_ONLY_COUT("");
			std::ofstream out;
			out.open(file_path);
			if (!out) {
				throw std::runtime_error("write file error!");
				return 0;
			}
			std::map<Point, int> point_map;
			int p_cnt = 0;
			for (int vi = 0, vi_end = cells_.size(); vi < vi_end; ++vi) {
				Cell* PC = cells_[vi];

				if (PC != NULL) {
					std::vector<Point> cell_points;
					std::vector<std::pair<int, int>> cell_edges;
					PC->output_wireframe(cell_points, cell_edges);

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

			return 1;
		}

		int output_cell_solid(
			std::string file_path,
			Cell& cell
		) {
			VERBOSE_ONLY_COUT("");
			std::ofstream out;
			out.open(file_path);
			if (!out) {
				throw std::runtime_error("write file error!");
				return 0;
			}

			std::vector<Point> cell_points;
			std::vector<std::vector<int>> cell_faces;
			cell.output_solid(cell_points, cell_faces);

			for (auto cp : cell_points) {
				out << "v" << " " << cp.x() << " " << cp.y() << " " << cp.z() << std::endl;
			}
			for (auto ce : cell_faces) {
				out << "f";
				for (auto ce_v : ce) {
					out << " " << ce_v + 1;
				}
				out << std::endl;
			}
			out.close();

			return 1;
		}

		int output_cells_solid(
			std::string file_path
		) {
			VERBOSE_ONLY_COUT("");
			std::ofstream out;
			out.open(file_path);
			if (!out) {
				throw std::runtime_error("write file error!");
				return 0;
			}

			std::map<Point, int> point_map;
			int p_cnt = 0;
			for (int vi = 0, vi_end = cells_.size(); vi < vi_end; ++vi) {
				Cell* PC = cells_[vi];

				if (PC != NULL) {
					std::vector<Point> cell_points;
					std::vector<std::vector<int>> cell_faces;
					PC->output_solid(cell_points, cell_faces);

					for (auto cp : cell_points) {
						if (point_map.find(cp) == point_map.end()) {
							out << "v" << " " << cp.x() << " " << cp.y() << " " << cp.z() << " " << "255 255 255" << std::endl;
							point_map[cp] = p_cnt++;
						}
					}
					for (auto cf : cell_faces) {
						out << "f";
						for (auto cfv : cf) {
							Point* P = &(cell_points[cfv]);
							out << " " << point_map[*P] + 1;
						}
						out << std::endl;
					}
					out << "v" << " " << PC->x() << " " << PC->y() << " " << PC->z() << " " << "255 0 0" << std::endl;
					p_cnt++;
				}
			}
			out.close();

			return 1;
		}

		int output_cells_connection(
			std::string file_path,
			OUTPUT_TYPE type
		) {
			VERBOSE_ONLY_COUT("");
			std::ofstream out;
			out.open(file_path);
			if (!out) {
				throw std::runtime_error("write file error!");
				return 0;
			}

			std::map<std::pair<int, int>, int> connect_map;
			for (int vi = 0, vi_end = cells_.size(); vi < vi_end; ++vi) {
				Cell* PC = cells_[vi];

				if (type == INFO) {
					out << vi << " ";
				}

				if (PC != NULL) {
					std::vector<CutPlane>* CP = &(PC->cutted_planes());

					std::vector<bool> plane_check(CP->size(), false);
					for (auto cv : PC->cutted_vertices()) {
						plane_check[std::get<0>(cv.first)] = true;
						plane_check[std::get<1>(cv.first)] = true;
						plane_check[std::get<2>(cv.first)] = true;
					}
					for (int i = 0, i_end = plane_check.size(); i < i_end; ++i) {
						if (plane_check[i]) {
							int op_i = (*CP)[i].opposite_id();
							if (op_i != -1) {
								std::pair<int, int> edge(std::min(vi, op_i), std::max(vi, op_i));
								if (connect_map.find(edge) == connect_map.end()) {
									connect_map[edge] = 0;
								}
								connect_map[edge]++;

								if (type == INFO) {
									out << op_i << " ";
								}
							}
						}
					}
				}

				if (type == INFO) {
					out << std::endl;
				}
			}
			if (type == VISUAL) {
				for (int i = 0, i_end = points_.size(); i < i_end; ++i) {
					out << "v" << " " << points_[i].x() << " " << points_[i].y() << " " << points_[i].z() << std::endl;
				}
				for (auto ep : connect_map) {
					if (ep.second == 2) {
						out << "l" << " " << ep.first.first + 1 << " " << ep.first.second + 1 << std::endl;
					}
				}
			}

			out.close();

			return 1;
		}
	private:
		double* get_bounding_box(const double redundancy) {
			VERBOSE_ONLY_COUT("");
			int points_nb = points_.size();

			double min_x = PD_MAX;
			double max_x = -PD_MAX;
			double min_y = PD_MAX;
			double max_y = -PD_MAX;
			double min_z = PD_MAX;
			double max_z = -PD_MAX;
			for (int i = 0; i < points_nb; ++i) {
				min_x = std::min(min_x, points_[i].x());
				max_x = std::max(max_x, points_[i].x());
				min_y = std::min(min_y, points_[i].y());
				max_y = std::max(max_y, points_[i].y());
				min_z = std::min(min_z, points_[i].z());
				max_z = std::max(max_z, points_[i].z());
			}

			double* res = new double[6];
			res[0] = min_x - redundancy;
			res[1] = max_x + redundancy;
			res[2] = min_y - redundancy;
			res[3] = max_y + redundancy;
			res[4] = min_z - redundancy;
			res[5] = max_z + redundancy;
			return res;
		}
	private:
		std::vector<Point> points_;
		std::vector<Cell*> cells_;
	};
} // namespace PowerDiagramGenerator

#endif

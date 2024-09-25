#ifndef POWER_DIAGRAM_GENERATOR_
#define POWER_DIAGRAM_GENERATOR_

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <tuple>
#include <list>
#include <map>
#include <unordered_set>
#include <string>

#ifndef EPS
#define EPS 1e-12
#endif

namespace PowerDiagramGenerator {
	struct pair_hash {
		template <class T1, class T2>
		size_t operator () (std::pair<T1, T2> const& pair) const {
			size_t h1 = hash<T1>()(pair.first);
			size_t h2 = hash<T2>()(pair.second);
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
		bool is_on_positive_side(Point p) const {
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
		if (d < EPS) {
			return Point();
		}
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

		void init_cube(double r) {
			cutted_planes_.emplace_back(CutPlane(cx_ + r, cy_, cz_, 1, 0, 0, -1));
			cutted_planes_.emplace_back(CutPlane(cx_ - r, cy_, cz_, -1, 0, 0, -1));
			cutted_planes_.emplace_back(CutPlane(cx_, cy_ + r, cz_, 0, 1, 0, -1));
			cutted_planes_.emplace_back(CutPlane(cx_, cy_ - r, cz_, 0, -1, 0, -1));
			cutted_planes_.emplace_back(CutPlane(cx_, cy_, cz_ + r, 0, 0, 1, -1));
			cutted_planes_.emplace_back(CutPlane(cx_, cy_, cz_ - r, 0, 0, -1, -1));

			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(0, 2, 4), Point(r, r, r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(2, 1, 4), Point(-r, r, r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(1, 3, 4), Point(-r, -r, r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(3, 0, 4), Point(r, -r, r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(2, 0, 5), Point(r, r, -r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(1, 2, 5), Point(-r, r, -r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(3, 1, 5), Point(-r, -r, -r)));
			cutted_vertices_.emplace_back(std::make_pair(std::tuple<int, int, int>(0, 3, 5), Point(r, -r, -r)));
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

		void output_Cell(
			std::list<Point>& points,
			std::list<std::pair<int, int>>& edges) const {
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
		std::vector<Point>& points) {
		points.clear();
		std::string back = file_path.substr(file_path.length() - 3, file_path.length());

		std::ifstream in(file_path);
		if (!in.good()) {
			throw "INPUT_FILE_PATH_INVALID";
			return 0;
		}
		if (back == "obj") { // .obj without weights
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
		else if (back == "xyz") { // .xyz
			std::string sline;
			while (std::getline(in, sline)) {
				std::istringstream ins(sline);
				Point p;
				ins >> p.x() >> p.y() >> p.z() >> p.w();
				points.push_back(p);
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
} // namespace PowerDiagramGenerator

#endif

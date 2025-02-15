#ifndef POWER_DIAGRAM_GENERATOR_HPP_
#define POWER_DIAGRAM_GENERATOR_HPP_

// #define POWER_DIAGRAM_GENERATOR_DEBUG
#ifdef POWER_DIAGRAM_GENERATOR_DEBUG
#define DEBUG_OUTPUT_PATH			"..//..//data//"
// #define OUTPUT_NEIGHBORS			"DEBUG_Neighbors"
// #define OUTPUT_CELLS_WIREFRAME	"DEBUG_Cells_Wireframe"
// #define OUTPUT_CELLS_SOLID		"DEBUG_Cells_Solid"
#endif

// #define POWER_DIAGRAM_GENERATOR_VERBOSE
#ifdef POWER_DIAGRAM_GENERATOR_VERBOSE
#	define VERBOSE_ONLY_COUT(x) std::cout << "[" << __FUNCTION__ << "]" << " " << x  << std::endl
#	define VERBOSE_ONLY_WARNING(x) std::cout << "[File:" << __FILE__ << "]" << "[Line:" << __LINE__ << "]" << " " << x << std::endl
#else
#	define VERBOSE_ONLY_COUT(x)
#	define VERBOSE_ONLY_WARNING(x)
#endif

#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_set>
#include <unordered_map>
#include <vector>
// Nanoflann
#include "nanoflann/nanoflann.hpp"
#include "nanoflann/utils.h"
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud<double> >, PointCloud<double>, 3> my_kd_tree_t;

#define PD_EPS 1e-12
#define PD_MAX 1e12
#define PD_SQRT_3 1.7320508075689

namespace PowerDiagramGenerator {
	/* use for unordered_map/unordered_set */
	struct pair_hash {
		template <class T1, class T2>
		size_t operator () (std::pair<T1, T2> const& pair) const {
			size_t h1 = std::hash<T1>()(pair.first);
			size_t h2 = std::hash<T2>()(pair.second);
			return h1 ^ h2;
		}
	};
	enum MODE {
		VORONOI_DIAGRAM		= 1,
		ENTIRE_BOUNDING_BOX = 1 << 1,
		EXACT_CALCULATION	= 1 << 2
	};

	class Point {
	public:
		Point() {};
		~Point() {};
		Point(double x,  double y, double z) {
			cor_[0] = x; cor_[1] = y; cor_[2] = z;
		}
		Point(double x, double y, double z, double w) {
			cor_[0] = x; cor_[1] = y; cor_[2] = z;
			w_ = w;
		}
		double& x() { return cor_[0]; }
		double& y() { return cor_[1]; }
		double& z() { return cor_[2]; }
		double* cor() { return cor_; }
		double& w() { return w_; }
		bool operator <(const Point& p) const {
			if (std::abs(cor_[0] - p.cor_[0]) < PD_EPS) {
				if (std::abs(cor_[1] - p.cor_[1]) < PD_EPS) {
					if (std::abs(cor_[2] - p.cor_[2]) < PD_EPS) {
						return (w_ < p.w_);
					}
					return (cor_[2] < p.cor_[2]);
				}
				return (cor_[1] < p.cor_[1]);
			}
			return (cor_[0] < p.cor_[0]);
		}
		bool operator ==(const Point& p) const {
			if (std::abs(cor_[0] - p.cor_[0]) < PD_EPS &&
				std::abs(cor_[1] - p.cor_[1]) < PD_EPS &&
				std::abs(cor_[2] - p.cor_[2]) < PD_EPS &&
				std::abs(w_ - p.w_) < PD_EPS) {
							return true;
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
		double sq_norm() {
			return cor_[0] * cor_[0] + cor_[1] * cor_[1] + cor_[2] * cor_[2];
		}
	protected:
		double cor_[3] = {0, 0, 0};
		double w_ = 0;
	};

	class Plane {
	public:
		Plane() {};
		~Plane() {};
		Plane(double cx, double cy, double cz,
			double nx, double ny, double nz) {
			a_ = nx; b_ = ny; c_ = nz;
			d_ = -cx * nx - cy * ny - cz * nz;
		}
		Plane(Point cor, Point nor) {
			a_ = nor.x(); b_ = nor.y(); c_ = nor.z();
			d_ = -cor.dot(nor);
		}
		double a() const { return a_; }
		double b() const { return b_; }
		double c() const { return c_; }
		double d() const { return d_; }

		double sign_distance(double x, double y, double z) const {
			return a_ * x + b_ * y + c_ * z + d_;
		}

		bool is_on_positive_side(double x, double y, double z) const {
			return (sign_distance(x, y, z) > PD_EPS);
		}
		bool is_on_positive_side(Point& p) const {
			return (sign_distance(p.x(), p.y(), p.z()) > PD_EPS);
		}
	protected:
		double a_ = 0, b_ = 0, c_ = 0, d_ = 0;
	};

	/** \brief Compute the intersecting point among three planes.
	* \param[in] P1: one of the three planes
	* \param[in] P2: one of the three planes
	* \param[in] P3: one of the three planes
	* \retval the intersecting point */
	Point get_tri_planes_cutted_point(const Plane& P1, const Plane& P2, const Plane& P3) {
		const double d = P1.a() * (P2.b() * P3.c() - P2.c() * P3.b()) - P1.b() * (P2.a() * P3.c() - P2.c() * P3.a()) + P1.c() * (P2.a() * P3.b() - P2.b() * P3.a());
		const double x = -P1.d() * (P2.b() * P3.c() - P2.c() * P3.b()) + P2.d() * (P1.b() * P3.c() - P1.c() * P3.b()) - P3.d() * (P1.b() * P2.c() - P1.c() * P2.b());
		const double y = P1.d() * (P2.a() * P3.c() - P2.c() * P3.a()) - P2.d() * (P1.a() * P3.c() - P1.c() * P3.a()) + P3.d() * (P1.a() * P2.c() - P1.c() * P2.a());
		const double z = P1.d() * (P2.b() * P3.a() - P2.a() * P3.b()) - P2.d() * (P1.b() * P3.a() - P1.a() * P3.b()) + P3.d() * (P1.b() * P2.a() - P1.a() * P2.b());
		return {x / d, y / d, z / d};
	}

	class CutPlane : public Plane {
	public:
		CutPlane(double cx, double cy, double cz,
			double nx, double ny, double nz,
			int opposite_id) {
			a_ = nx; b_ = ny; c_ = nz;
			d_ = -cx * nx - cy * ny - cz * nz;
			opposite_id_ = opposite_id;
		}
		CutPlane(Point cor, Point nor,
			int opposite_id) {
			a_ = nor.x(); b_ = nor.y(); c_ = nor.z();
			d_ = -cor.dot(nor);
			opposite_id_ = opposite_id;
		}
		int& opposite_id() { return opposite_id_; }
	protected:
		int opposite_id_;
	};

	class Cell {
	public:
		Cell() {};
		~Cell() {
			(std::vector<CutPlane>()).swap(cutted_planes_);
			(std::vector<std::pair<std::tuple<int, int, int>, Point>>()).swap(cutted_vertices_);
		}
		Cell(const double cx, const double cy, const double cz) {
			cx_ = cx; cy_ = cy; cz_ = cz;
		}
		Cell(Point& p) {
			cx_ = p.x(); cy_ = p.y(); cz_ = p.z();
		}
		double x() { return cx_; }
		double y() { return cy_; }
		double z() { return cz_; }
		std::vector<CutPlane>& cutted_planes() { return cutted_planes_; }
		const std::vector<std::pair<std::tuple<int, int, int>, Point>>& cutted_vertices() { return cutted_vertices_; }
		double sq_dist() {return sq_dist_;}

		/** \brief Get the farthest distance between the cell's vertices and the center point.
		* \retval the squared distance */
		double get_farthest_vertex_sq_distance() {
			double dist = -PD_MAX;
			for (int i = 0, i_end = cutted_vertices_.size(); i < i_end; i++) {
				dist = std::max(dist, (Point(cx_, cy_, cz_) - cutted_vertices_[i].second).sq_norm());
			}
			return dist;
		}

		/** \brief Initialize the bounding box of this cell to a box.
		* \param[in] r: the radius of the box */
		void init_cube(const double r) {
			cutted_planes_.emplace_back(cx_ + r, cy_, cz_, 1, 0, 0, -1);
			cutted_planes_.emplace_back(cx_ - r, cy_, cz_, -1, 0, 0, -1);
			cutted_planes_.emplace_back(cx_, cy_ + r, cz_, 0, 1, 0, -1);
			cutted_planes_.emplace_back(cx_, cy_ - r, cz_, 0, -1, 0, -1);
			cutted_planes_.emplace_back(cx_, cy_, cz_ + r, 0, 0, 1, -1);
			cutted_planes_.emplace_back(cx_, cy_, cz_ - r, 0, 0, -1, -1);

			cutted_vertices_.emplace_back(std::tuple<int, int, int>(0, 2, 4), Point(cx_ + r, cy_ + r, cz_ + r));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(2, 1, 4), Point(cx_ - r, cy_ + r, cz_ + r));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(1, 3, 4), Point(cx_ - r, cy_ - r, cz_ + r));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(3, 0, 4), Point(cx_ + r, cy_ - r, cz_ + r));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(2, 0, 5), Point(cx_ + r, cy_ + r, cz_ - r));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(1, 2, 5), Point(cx_ - r, cy_ + r, cz_ - r));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(3, 1, 5), Point(cx_ - r, cy_ - r, cz_ - r));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(0, 3, 5), Point(cx_ + r, cy_ - r, cz_ - r));

			sq_dist_ = get_farthest_vertex_sq_distance();
		}

		/** \brief Initialize the bounding box of this cell to a specified box.
		* \param[in] min_x: the x-component of a corner vertex of the box
		* \param[in] min_y: the y-component of a corner vertex of the box
		* \param[in] min_z: the z-component of a corner vertex of the box
		* \param[in] max_x: the x-component another corner vertex of the box
		* \param[in] max_y: the y-component another corner vertex of the box
		* \param[in] max_z: the z-component another corner vertex of the box */
		void init_box(
			const double min_x, const double min_y, const double min_z,
			const double max_x, const double max_y, const double max_z) {
			double center_x = 0.5 * (min_x + max_x);
			double center_y = 0.5 * (min_y + max_y);
			double center_z = 0.5 * (min_z + max_z);

			cutted_planes_.emplace_back(max_x, center_y, center_z, 1, 0, 0, -1);
			cutted_planes_.emplace_back(min_x, center_y, center_z, -1, 0, 0, -1);
			cutted_planes_.emplace_back(center_x, max_y, center_z, 0, 1, 0, -1);
			cutted_planes_.emplace_back(center_x, min_y, center_z, 0, -1, 0, -1);
			cutted_planes_.emplace_back(center_x, center_y, max_z, 0, 0, 1, -1);
			cutted_planes_.emplace_back(center_x, center_y, min_z, 0, 0, -1, -1);

			cutted_vertices_.emplace_back(std::tuple<int, int, int>(0, 2, 4), Point(max_x, max_y, max_z));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(2, 1, 4), Point(min_x, max_y, max_z));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(1, 3, 4), Point(min_x, min_y, max_z));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(3, 0, 4), Point(max_x, min_y, max_z));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(2, 0, 5), Point(max_x, max_y, min_z));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(1, 2, 5), Point(min_x, max_y, min_z));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(3, 1, 5), Point(min_x, min_y, min_z));
			cutted_vertices_.emplace_back(std::tuple<int, int, int>(0, 3, 5), Point(max_x, min_y, min_z));

			sq_dist_ = get_farthest_vertex_sq_distance();
		}

		/** \brief Initialize the bounding box of this cell to a specified box.
		* \param[in] P1: the point of a corner vertex of the box
		* \param[in] P2: the point of another corner vertex of the box */
		void init_box(Point& P1, Point& P2) {
			const double min_x = std::min(P1.x(), P2.x());
			const double min_y = std::min(P1.y(), P2.y());
			const double min_z = std::min(P1.z(), P2.z());
			const double max_x = std::max(P1.x(), P2.x());
			const double max_y = std::max(P1.y(), P2.y());
			const double max_z = std::max(P1.z(), P2.z());
			init_box(min_x, min_y, min_z, max_x, max_y, max_z);
		}

		/** \brief Cut this cell by a plane.
		 * \param[in] plane: the plane used to cut this cell
		 * \retval does the plane cut through this cell */
		bool cut_by_plane(const CutPlane& plane) {
			// find cut vertices
			std::vector<std::tuple<int, int, int>> cut_vertices; // the cut vertices in cut_vertices_
			std::vector<std::pair<std::tuple<int, int, int>, Point>> cut_vertices_tmp; // tmp for cut_vertices_
			for (auto & cut_vertex : cutted_vertices_) {
				if (plane.is_on_positive_side(cut_vertex.second)) {
					cut_vertices.push_back(cut_vertex.first);
				}
				else {
					cut_vertices_tmp.push_back(cut_vertex);
				}
			}
			cutted_vertices_.swap(cut_vertices_tmp);

			if (cut_vertices.empty()) return false;

			// find cut edge to get the real cut vertices
			std::unordered_set<int> remain_planes_set; // the real remained planes of this cell
			for (auto & cut_vertex : cutted_vertices_) {
				remain_planes_set.insert(std::get<0>(cut_vertex.first));
				remain_planes_set.insert(std::get<1>(cut_vertex.first));
				remain_planes_set.insert(std::get<2>(cut_vertex.first));
			}
			std::unordered_set<std::pair<int, int>, pair_hash> cut_edges;
			for (auto & cut_vertex : cut_vertices) {
				std::pair<int, int> edge1 = std::make_pair(std::get<0>(cut_vertex), std::get<1>(cut_vertex));
				std::pair<int, int> edge2 = std::make_pair(std::get<1>(cut_vertex), std::get<2>(cut_vertex));
				std::pair<int, int> edge3 = std::make_pair(std::get<2>(cut_vertex), std::get<0>(cut_vertex));

				auto f_e1 = cut_edges.find(std::make_pair(std::get<1>(cut_vertex), std::get<0>(cut_vertex)));
				if (f_e1 == cut_edges.end()) {
					cut_edges.insert(edge1);
				}
				else {
					cut_edges.erase(f_e1);
				}

				auto f_e2 = cut_edges.find(std::make_pair(std::get<2>(cut_vertex), std::get<1>(cut_vertex)));
				if (f_e2 == cut_edges.end()) {
					cut_edges.insert(edge2);
				}
				else {
					cut_edges.erase(f_e2);
				}

				auto f_e3 = cut_edges.find(std::make_pair(std::get<0>(cut_vertex), std::get<2>(cut_vertex)));
				if (f_e3 == cut_edges.end()) {
					cut_edges.insert(edge3);
				}
				else {
					cut_edges.erase(f_e3);
				}
			}

			// process
			std::list<int> dangling_planes;
			while (!cut_edges.empty()) {
				int cur_cut_edges_nb = cut_edges.size();

				for (auto it = cut_edges.begin(); it != cut_edges.end();) {
					if (dangling_planes.empty()) {
						dangling_planes.push_back((*it).first);
						dangling_planes.push_back((*it).second);
						it = cut_edges.erase(it);
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

						it = cut_edges.erase(it);
					}
				}

				if (cut_edges.size() == cur_cut_edges_nb) break;
			}

			// renew
			if (dangling_planes.size() > 2) {
				cutted_planes_.push_back(plane);

				int planes_cnt = 0;
				auto prev_it = dangling_planes.end();
				prev_it--;
				for (auto it = dangling_planes.begin(); it != dangling_planes.end(); ++it) {
					Plane* plane1 = &(cutted_planes_[(*prev_it)]);
					Plane* plane2 = &(cutted_planes_[(*it)]);

					Point cutted_point = get_tri_planes_cutted_point((*plane1), (*plane2), plane);
					if (std::isnan(cutted_point.x()) || std::isnan(cutted_point.y()) || std::isnan(cutted_point.z()) ||
						std::isinf(cutted_point.x()) || std::isinf(cutted_point.y()) || std::isinf(cutted_point.z())) {
						// do nothing
					}
					else {
						cutted_vertices_.emplace_back(
							std::make_tuple((*prev_it), (*it), cutted_planes_.size() - 1),
								cutted_point);
					}
					prev_it = it;
				}
			}

			sq_dist_ = get_farthest_vertex_sq_distance();

			return true;
		}

		/** \brief Output this cell's polyhedral wireframe.
		 * \param[out] points: cell's vertices
		 * \param[out] edges: connection between points */
		void output_wireframe(
			std::vector<Point>& points,
			std::vector<std::pair<int, int>>& edges) const {
			std::map<std::pair<int, int>, std::vector<int>> plane_pair_and_vertices;
			for (int i = 0, i_end = cutted_vertices_.size(); i < i_end; ++i) {
				int a = std::get<0>(cutted_vertices_[i].first);
				int b = std::get<1>(cutted_vertices_[i].first);
				int c = std::get<2>(cutted_vertices_[i].first);
				std::pair<int, int> plane_pair1(std::min(a, b), std::max(a, b));
				std::pair<int, int> plane_pair2(std::min(b, c), std::max(b, c));
				std::pair<int, int> plane_pair3(std::min(c, a), std::max(c, a));

				plane_pair_and_vertices[plane_pair1].push_back(i);
				plane_pair_and_vertices[plane_pair2].push_back(i);
				plane_pair_and_vertices[plane_pair3].push_back(i);
			}

			std::vector<Point>().swap(points);
			for (int i = 0, i_end = cutted_vertices_.size(); i < i_end; ++i) {
				points.push_back(cutted_vertices_[i].second);
			}

			std::vector<std::pair<int, int>>().swap(edges);
			for (auto pv = plane_pair_and_vertices.begin(); pv != plane_pair_and_vertices.end(); ++pv) {
				if (pv->second.size() == 2) {
					edges.emplace_back(pv->second[0], pv->second[1]);
				}
			}
		}

		/** \brief Output this cell's polyhedron.
		 * \param[out] points: cell's vertices
		 * \param[out] faces: polyhedron's polyhedral face */
		void output_polyhedron(
			std::vector<Point>& points,
			std::vector<std::vector<int>>& faces
			) const {
			std::unordered_map<std::pair<int, int>, int, pair_hash> edge_to_vertex_map; // edge<plane1, plane2> -> the vertex unique to this ordered edge
			std::unordered_map<std::pair<int, int>, int, pair_hash> edge_to_plane_map; // edge<plane1, plane2> -> the remain plane intersected at the same vertex
			std::unordered_map<int, std::vector<int>> plane_neighbor_planes; // plane -> the planes adjacent to this plane
			std::vector<std::unordered_set<int>> neighbor_planes(cutted_planes_.size());

			std::vector<Point>().swap(points);
			for (const auto & cutted_vertice : cutted_vertices_) {
				const int P1 = std::get<0>(cutted_vertice.first);
				const int P2 = std::get<1>(cutted_vertice.first);
				const int P3 = std::get<2>(cutted_vertice.first);
				const std::pair<int, int> edge12(P1, P2);
				const std::pair<int, int> edge23(P2, P3);
				const std::pair<int, int> edge31(P3, P1);

				edge_to_vertex_map[edge12] = points.size();
				edge_to_vertex_map[edge23] = points.size();
				edge_to_vertex_map[edge31] = points.size();
				edge_to_plane_map[edge12] = P3;
				edge_to_plane_map[edge23] = P1;
				edge_to_plane_map[edge31] = P2;

				if (plane_neighbor_planes.find(P1) == plane_neighbor_planes.end()) {
					plane_neighbor_planes[P1] = std::vector<int>();
				}
				if (plane_neighbor_planes.find(P2) == plane_neighbor_planes.end()) {
					plane_neighbor_planes[P2] = std::vector<int>();
				}
				if (plane_neighbor_planes.find(P3) == plane_neighbor_planes.end()) {
					plane_neighbor_planes[P3] = std::vector<int>();
				}
				plane_neighbor_planes[P1].push_back(P2);
				plane_neighbor_planes[P2].push_back(P3);
				plane_neighbor_planes[P3].push_back(P1);

				points.push_back(cutted_vertice.second);
			}

			// get each face's edges, choose Pi, then traver the plane Pj adjacent to Pi
			std::vector<std::vector<int>>().swap(faces);
			for (auto & f_nei_fs : plane_neighbor_planes) {
				int Pi = f_nei_fs.first;
				std::vector<int> face;

				int Pj = f_nei_fs.second[0];
				for (auto & nothing_to_do : f_nei_fs.second) {
					std::pair<int, int> cur_edge(Pi, Pj);
					face.push_back(edge_to_vertex_map[cur_edge]);
					Pj = edge_to_plane_map[cur_edge];
				}

				faces.push_back(face);
			}
		}

	protected:
		double cx_ = 0, cy_ = 0, cz_ = 0;
		std::vector<CutPlane> cutted_planes_; /* The plane that has cut this cell.
												Note that not all planes now still cut this cell*/
		std::vector<std::pair<std::tuple<int, int, int>, Point>> cutted_vertices_; /* The vertices of this polyhedron cell.
								The first->tuple indicates which three planes (ids) in cutted_planes() intersect this point
								The second->Point indicates this point */
		double sq_dist_ = 0; /* The farthest distance between the cell's vertices and the center point. */
	};

	class Generator {
	public:
		Generator() {};
		~Generator() {
			std::vector<Point>().swap(points_);
			std::vector<Cell>().swap(cells_);
		}
		Cell& cell(const int i) {
			return cells_[i];
		}

		/** \brief Get input point cloud.
		 * \details If the input file type is .xyz, it should be in the format of "x y z w";
		 *			If the input file type is .obj, it does not support reading points' weights;
		 *			If the input file type is .off, it does not support reading points' weights;
		 * \param[in] file_path: input point cloud's file path
		 * \retval success or not */
		int read_pointcloud(const std::string file_path) {
			VERBOSE_ONLY_COUT("reading:" << file_path);
			(std::vector<Point>()).swap(points_);
			std::string back = file_path.substr(file_path.length() - 3);

			std::ifstream in(file_path);
			if (!in.good()) {
				VERBOSE_ONLY_WARNING("reading input file error!");
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
				VERBOSE_ONLY_WARNING("input file type is invalid!");
				return 0;
			}
			in.close();

			return 1;
		}

		/** \brief [DISUSE] Get the points' neighbor points.
		 * \param[in] sq_radis: squared distance used for neighbor searching
		 * \param[out] neighbors: points' neighbors' ids
		 * \retval success or not */
		int get_neighbors(
			const double sq_radis,
			std::vector<std::vector<int>>& neighbors) { // disuse
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
				std::ofstream out(std::string(DEBUG_OUTPUT_PATH) + std::string(OUTPUT_NEIGHBORS) + ".obj");
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

		/** \brief Generate the Power/Voronoi diagram.
		 * \param[in] radius: the radius of the diagram cell; In ENTIRE_BOUNDING_BOX mode, it is used to set the extended length of the bounding box
		 * \param[in] mode: "VORONOI_DIAGRAM" compute the Voronoi diagram;
		 *					"ENTIRE_BOUNDING_BOX" set the initial box to entire bounding box;
		 *					"EXACT_CALCULATION" complete traverse all points to calculate, only for verification
		 * \retval success or not */
		int generate_diagram(
			const double radius,
			const int mode) {
			VERBOSE_ONLY_COUT("");

			if (radius < PD_EPS) {
				VERBOSE_ONLY_WARNING("radius=" << radius << " " << "setting is invalid!");
				return 0;
			}

			// init
			cells_ = std::vector<Cell>(points_.size());

			// get bounding box (only for ENTIRE_BOUNDING_BOX mode)
			double* box = new double[6];
			if (mode & ENTIRE_BOUNDING_BOX) box = get_bounding_box(radius);

			// get maximum and minimum weights
			double max_w = -PD_MAX, min_w = PD_MAX;
			for (int i = 0, i_end = points_.size(); i < i_end; ++i) {
				max_w = std::max(max_w, points_[i].w());
				min_w = std::min(min_w, points_[i].w());
			}

			// build kd-tree cloud
			PointCloud<double> cloud;
			cloud.pts.resize(points_.size());
			for (int i = 0, i_end = points_.size(); i < i_end; ++i) {
				cloud.pts[i].x = points_[i].x();
				cloud.pts[i].y = points_[i].y();
				cloud.pts[i].z = points_[i].z();
			}
			const my_kd_tree_t index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));

			const int points_nb = points_.size();
#pragma omp parallel for
			for (int vi = 0; vi < points_nb; ++vi) {
				nanoflann::SearchParams params;
				if (vi % 5000 == 0) VERBOSE_ONLY_COUT("process cell cnt:" << " " << vi);

				// init the cell
				cells_[vi] = Cell(points_[vi]);
				Cell* PC = &(cells_[vi]);

				if (mode & ENTIRE_BOUNDING_BOX) {
					PC->init_box(box[0], box[1], box[2], box[3], box[4], box[5]);
				}
				else {
					PC->init_cube(radius);
				}

				// get maximum search radius
				double sq_search_radius = PC->sq_dist() - points_[vi].w() + max_w;

				// get neighbors
				std::vector<std::pair<uint32_t, double>> ret_matches;
				const size_t nMatches = index.radiusSearch(points_[vi].cor(), sq_search_radius, ret_matches, params);

				// process each neighbors
				for (int j = 1; j < nMatches; ++j) {
					const int ni = ret_matches[j].first; // the id of the neighbors' point

					const double sq_dis = ret_matches[j].second;
					//	(points_[vi].x() - points_[ni].x()) * (points_[vi].x() - points_[ni].x()) +
					//	(points_[vi].y() - points_[ni].y()) * (points_[vi].y() - points_[ni].y()) +
					//	(points_[vi].z() - points_[ni].z()) * (points_[vi].z() - points_[ni].z());
					// if (sq_dis < PD_EPS) continue;

					double lambda;
					if (mode & VORONOI_DIAGRAM) {
						lambda = 0.5;
					}
					else {
						lambda = 0.5 + 0.5 * (points_[vi].w() * points_[vi].w() - points_[ni].w() * points_[ni].w()) / sq_dis;
					}

					const Point mid_point = (points_[vi] * lambda) + (points_[ni] * (1 - lambda));
					const Point dir = points_[ni] - points_[vi];

					if (PC->cut_by_plane(
						CutPlane(
						mid_point, dir, ni)) == false) {
						if ((mode & EXACT_CALCULATION == false) &&
							(mode & VORONOI_DIAGRAM)) break; // the remaining points are not need to judge
					}
					if ((mode & EXACT_CALCULATION == false) &&
						(mode & VORONOI_DIAGRAM == false)) { // power diagram mode
						sq_search_radius = PC->sq_dist() - points_[vi].w() + max_w;
						if (ret_matches[j].second > sq_search_radius) break; // the remaining points are not need to judge
					}
				}
			}

			return 1;
		}

		/** \brief Output a cell' polyhedron's wireframe (OBJ format).
		* \param[in] out: the output stream
		* \param[in] cell: the cell that want to output
		* \param[in, out] start_id: the start vertices id, and output the id that should start next time */
		void output_cell_wireframe(
			std::ofstream& out,
			Cell& cell,
			int& start_id) {
			std::vector<Point> cell_points;
			std::vector<std::pair<int, int>> cell_edges;
			cell.output_wireframe(cell_points, cell_edges);

			for (auto cp_it = cell_points.begin(); cp_it != cell_points.end(); ++cp_it) {
				out << "v" << " " << cp_it->x() << " " << cp_it->y() << " " << cp_it->z() << " " << "255 255 255" << std::endl;
			}
			for (auto ce_it = cell_edges.begin(); ce_it != cell_edges.end(); ++ce_it) {
				out << "l" << " " << start_id + ce_it->first + 1 << " " << start_id + ce_it->second + 1 << std::endl;
			}
			out << "v" << " " << cell.x() << " " << cell.y() << " " << cell.z() << " " << "255 0 0" << std::endl; // the center point of this cell

			start_id += cell_points.size() + 1; // need to add the addition center point
		}

		/** \brief Output cells' polyhedrons' wireframe (OBJ format).
		* \param[in] out: the output stream */
		void output_cells_wireframe(
			std::ofstream& out) {
			VERBOSE_ONLY_COUT("");

			std::map<Point, int> point_id_map;
			int p_cnt = 0;
			for (auto & PC : cells_) {
				if (&PC != nullptr) {
					std::vector<Point> cell_points;
					std::vector<std::pair<int, int>> cell_edges;
					PC.output_wireframe(cell_points, cell_edges);

					std::map<int, int> edge_vertex_id_map;
					for (size_t i = 0, i_end = cell_points.size(); i < i_end; ++i) {
						Point* p = &(cell_points[i]);
						if (point_id_map.find(*p) == point_id_map.end()) {
							point_id_map[*p] = p_cnt++;
							out << "v" << " " << p->x() << " " << p->y() << " " << p->z() << " " << "255 255 255" << std::endl;
						}
						edge_vertex_id_map[i] = point_id_map[*p];
					}
					for (auto & e : cell_edges) {
						out << "l" << " " << edge_vertex_id_map[e.first] + 1 << " " << edge_vertex_id_map[e.second] + 1 << std::endl;
					}

					// draw cell's center point
					out << "v" << " " << PC.x() << " " << PC.y() << " " << PC.z() << " " << "255 0 0" << std::endl;
					++p_cnt;
				}
			}
		}

		/** \brief Output a cell' polyhedron (OBJ format).
		* \param[in] out: the output stream
		* \param[in] cell: the cell that want to output
		* \param[in, out] start_id: the start vertices id, and output the id that should start next time */
		void output_cell_polyhedron(
			std::ofstream& out,
			const Cell& cell,
			int& start_id) {
			std::vector<Point> cell_points;
			std::vector<std::vector<int>> cell_faces;
			cell.output_polyhedron(cell_points, cell_faces);

			for (auto & cell_point : cell_points) {
				out << "v" << " " << cell_point.x() << " " << cell_point.y() << " " << cell_point.z() << std::endl;
			}
			for (auto & cell_face : cell_faces) {
				out << "f";
				for (const int & ce_v_it : cell_face) {
					out << " " << start_id + ce_v_it + 1;
				}
				out << std::endl;
			}
			start_id += cell_points.size();
		}

		/** \brief Output cells' polyhedrons (OBJ format).
		* \param[in] out: the output stream
		* \param[in, out] start_id: the start vertices id, and output the id that should start next time */
		void output_cells_polyhedron(
			std::ofstream& out,
			int& start_id) {
			VERBOSE_ONLY_COUT("");

			for (auto & cell : cells_) output_cell_polyhedron(out, cell, start_id);
		}

		/** \brief Output Power/Voronoi diagram's info.
		* \param[in] out: the output stream
		* \param[in] output_type: output format,
		*						"neighbor" output the connection relationship of each point in the diagram, output in the form of "i neighbor_1 neighbor_2 ..."
		*						"connection" visually output the connection between points */
		void output_cells_info(
			std::ofstream& out,
			const std::string& output_type) {
			VERBOSE_ONLY_COUT("");

			std::map<std::pair<int, int>, int> connect_map;
			for (int vi = 0, vi_end = cells_.size(); vi < vi_end; ++vi) {
				Cell* PC = &(cells_[vi]);

				if (output_type == "neighbor") {
					out << vi << " ";
				}

				if (PC != nullptr) {
					std::vector<CutPlane>* CP = &(PC->cutted_planes());

					std::vector<bool> plane_check(CP->size(), false);
					for (auto cv_it = PC->cutted_vertices().begin(); cv_it != PC->cutted_vertices().end(); ++cv_it) {
						plane_check[std::get<0>(cv_it->first)] = true;
						plane_check[std::get<1>(cv_it->first)] = true;
						plane_check[std::get<2>(cv_it->first)] = true;
					}
					for (size_t i = 0, i_end = plane_check.size(); i < i_end; ++i) {
						if (plane_check[i]) {
							if (int op_i = (*CP)[i].opposite_id(); op_i != -1) {
								std::pair<int, int> edge(std::min(vi, op_i), std::max(vi, op_i));
								if (connect_map.find(edge) == connect_map.end()) {
									connect_map[edge] = 0;
								}
								connect_map[edge]++;

								if (output_type == "neighbor") {
									out << op_i << " ";
								}
							}
						}
					}
				}

				if (output_type == "neighbor") {
					out << std::endl;
				}
			}
			if (output_type == "connection") {
				for (int i = 0, i_end = points_.size(); i < i_end; ++i) {
					out << "v" << " " << points_[i].x() << " " << points_[i].y() << " " << points_[i].z() << std::endl;
				}
				for (auto ep_it = connect_map.begin(); ep_it != connect_map.end(); ++ep_it) {
					if (ep_it->second == 2) {
						out << "l" << " " << ep_it->first.first + 1 << " " << ep_it->first.second + 1 << std::endl;
					}
				}
			}
		}

		/** \brief Get the input points' bounding box with redundancy.
		* \param[in] redundancy: the length to expand the bounding box outward
		* \retval the bounding box's {min_x, min_y, min_z, max_x, max_y, max_z} */
		double* get_bounding_box(const double redundancy) {
			VERBOSE_ONLY_COUT("");

			double min_x = PD_MAX;
			double max_x = -PD_MAX;
			double min_y = PD_MAX;
			double max_y = -PD_MAX;
			double min_z = PD_MAX;
			double max_z = -PD_MAX;
			for (int i = 0, i_end = points_.size(); i < i_end; ++i) {
				min_x = std::min(min_x, points_[i].x());
				max_x = std::max(max_x, points_[i].x());
				min_y = std::min(min_y, points_[i].y());
				max_y = std::max(max_y, points_[i].y());
				min_z = std::min(min_z, points_[i].z());
				max_z = std::max(max_z, points_[i].z());
			}

			double* res = new double[6];
			res[0] = min_x - redundancy;
			res[1] = min_y - redundancy;
			res[2] = min_z - redundancy;
			res[3] = max_x + redundancy;
			res[4] = max_y + redundancy;
			res[5] = max_z + redundancy;
			return res;
		}

	private:
		std::vector<Point> points_; // input points
		std::vector<Cell> cells_; // output cells
	};
} // namespace PowerDiagramGenerator

#endif

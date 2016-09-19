#include <cassert>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <fstream>
#include <CGAL/box_intersection_d.h>
#include "Intersections.h"

typedef CGAL::Exact_predicates_exact_constructions_kernel  Kernel;
typedef CGAL::Simple_cartesian<double> M;
typedef M::Point_3         Point_3;
typedef M::Segment_3       Segment_3;
typedef M::Triangle_3      Triangle;
typedef M::Intersect_3 Intersect_3;
typedef std::vector<Triangle>                               Triangles;
typedef Triangles::iterator                                   Iterator;
typedef CGAL::Box_intersection_d::Box_with_handle_d<double, 3, Iterator> Box;
typedef std::vector<Box>                               Boxes;
typedef CGAL::cpp11::result_of<Intersect_3(Triangle, Triangle)>::type type;

enum planes {Parallel, Equal, Intersect};
enum triangles{No_intersection, Touch, Intersect};
enum where_intersect{No_intersect, Inside, Edge, Border};

struct polyline_elements {
	type elem;
	planes p;
	triangles t;
	where_intersect w;
};

std::vector<polyline_elements> results;

class Intersect_result {
	type res;
	planes p;
	triangles t;
	where_intersect w;
public:
	Intersect_result()
	{}
	void operator()(const Box &a, const Box &b) {
		Triangle t1 = *a.handle();
		Triangle t2 = *b.handle();
		CGAL::cpp11::result_of<Intersect_3(M::Plane_3, M::Plane_3)>::type plane_intersect = intersection(t1.supporting_plane(), t2.supporting_plane());
		if (!plane_intersect) {
			p = Parallel;
			t = No_intersection;
			w = No_intersect;
		}
		if (const M::Plane_3* pl = boost::get<M::Plane_3>(&*plane_intersect)) {
			p = Equal;

		}
		if (type result = intersect(t1, t2))
			results.push_back({result, p, t, w});
		}
};

void print(std::vector<polyline_elements> result) {
	for (polyline_elements i : result) {
		if (i.elem) {
			std::cout << "intersection" << std::endl;
			if (const Segment_3* s = boost::get<Segment_3>(&*i.elem))
				std::cout << "Segment " << *s << std::endl;
			if (const Point_3* p = boost::get<Point_3>(&*i.elem))
				std::cout << "Point " << *p << std::endl;
			if (const Triangle* tr = boost::get<Triangle>(&*i.elem))
				std::cout << "Triangle " << *tr << std::endl;
			if (const std::vector<Point_3>* st = boost::get<std::vector<Point_3>>(&*i.elem)) {
				std::cout << "Set of points:" << std::endl;
				for (std::vector<Point_3>::const_iterator it = st->begin(); it != st->end(); ++it)
					std::cout << *it << ' ';
				std::cout << std::endl;
			}
		}
		else
			std::cout << "no intersection" << std::endl;
	}
}

int main(int argc, char* argv[]) {
	std::ifstream first(argv[1]);
	std::ifstream second(argv[2]);
	Triangles tr_1;
	Triangles tr_2;
	Triangle tr;
	while (first >> tr)
		if (tr.is_degenerate())
			std::cout << "Attention! Triangle " << tr << " is degenerate!" << std::endl;
		else
			tr_1.push_back(tr);
	while (second >> tr) 
		if (tr.is_degenerate())
			std::cout << "Attention! Triangle " << tr << " is degenerate!" << std::endl;
		else
			tr_2.push_back(tr);
	Boxes b_1;
	Boxes b_2;
	for (std::vector<Triangle>::iterator i = tr_1.begin(); i != tr_1.end(); ++i) {
		b_1.push_back(Box(i->bbox(), i));
	}
	for (std::vector<Triangle>::iterator i = tr_2.begin(); i != tr_2.end(); ++i) {
		b_2.push_back(Box(i->bbox(), i));
	}
	CGAL::box_intersection_d(b_1.begin(), b_1.end(), b_2.begin(), b_2.end(), Intersect_result());
	print(results);
	return 0;
}

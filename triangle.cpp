#include "geom_obj.hpp"

#define TEST_INTER_PLANES 0

g_obj::triangle_t get_triangle() {
    std::vector<g_obj::point_t> verts(3);
    g_obj::point_t temp;
    for (int i = 0; i < 3; ++i) {
        std::cin >> temp.x;
        std::cin >> temp.y;
        std::cin >> temp.z; 
        verts[i] = temp;
        assert(std::cin.good());
    }

    g_obj::plane_t plane_{verts[0], verts[1], verts[2]};

    std::vector<g_obj::line_segment> section_arr(3);
    for (int i = 0; i < 3; ++i) {
        section_arr[i] = {verts[i], verts[(i+1) % 3]};
    }
    std::vector<g_obj::line_t> line_arr(3);
    for (int i = 0; i < 3; ++i) {
        line_arr[i] = section_arr[i].get_line();
    }

    g_obj::triangle_t ret{verts, plane_, line_arr};
    //ret.print();
    return ret;
}

std::vector<g_obj::point_t> find_inter_points(const g_obj::triangle_t &tr,
                                              const g_obj::line_t &inter_line) {

    std::vector<g_obj::point_t> inter_points(2);
    int count_inters{0};

    for (int i = 0; i < 3 && count_inters < 2; i++) {
        if (inter_line.parallelism(tr.lines[i])) {            //if lines are parallel
            if (inter_line.point_belong(tr.vertices[i]) &&
                inter_line.point_belong(tr.vertices[(i+1)%3])) {
                inter_points[0] = tr.vertices[i]; inter_points[1] = tr.vertices[(i+1)%3];
                count_inters = 2;
                break;
            }
        }
        else {   //if intersections
            inter_points[count_inters] = inter_line.point_of_intersect(tr.lines[i]);
            count_inters++;
            if (count_inters == 2) break;
        }
    }

    if (count_inters != 2) {
        std::cerr << "ERROR: incorrect number of points of intersection" << std::endl;
        exit(EXIT_FAILURE);
    }

    return inter_points;
}

bool check_point_belongs(const std::vector<g_obj::point_t> &inter_points, const g_obj::point_t &point) {
    if (inter_points.size() != 2) {
        std::cerr << "ERROR: incorrect number of points of intersection" << std::endl;
        exit(EXIT_FAILURE);
    }

    g_obj::line_segment section{inter_points[0], inter_points[1]};
    float len_section = section.len();
    if (point.distance(inter_points[0]) <= len_section &&
        point.distance(inter_points[1]) <= len_section)
        return true;
    return false;
}

bool check_tr_inter(const g_obj::triangle_t &tr1, const g_obj::triangle_t &tr2) {
    g_obj::line_t inter_line = tr1.plane.line_of_intersect(tr2.plane);
    
    std::vector<g_obj::point_t> inter_points1(2), inter_points2(2);

    inter_points1 = find_inter_points(tr1, inter_line);
    inter_points2 = find_inter_points(tr2, inter_line);

    if (inter_points1[0].equal(inter_points1[1])) {
        if (inter_points2[0].equal(inter_points2[1])) {
            if (inter_points1[0].equal(inter_points2[0]))
                return true;
            else return false;
        }
        else if (check_point_belongs(inter_points2, inter_points1[0]))
            return true;
        else return false;
    }

    for (int i = 0; i < 2; i++) {
        if (check_point_belongs(inter_points1, inter_points2[i]))
            return true;
        else return false;        
    }

    return false;
}

int main(int argc, char** argv) 
{
    using g_obj::point_t;
    using g_obj::plane_t;
    using g_obj::triangle_t;

    int N;
    std::cin >> N;

    std::vector<triangle_t> triangles(N);
    for (int i = 0; i < N; i++) {
        triangles[i] = get_triangle();
    }

    for (int i = 0; i < N; i++) {
        if (triangles[i].inter) {
            std::cout << i << std::endl;
            continue;
        } else {
            for (int j = i + 1; j < N; j++) {
                if (check_tr_inter(triangles[i], triangles[j])) {
                    triangles[i].inter = true;
                    triangles[j].inter = true;
                }
            }
        }
    }

    for (int i = 0; i < N; i++) {
        if (triangles[i].inter == true)
            std::cout << i << std::endl;
    }
    
    #if TEST_INTER_PLANES
    point_t  p1, p2, p3;
    point_t  p4, p5, p6;

    std::cin >> p1.x;
    std::cin >> p1.y;
    std::cin >> p1.z;

    std::cin >> p2.x;
    std::cin >> p2.y;
    std::cin >> p2.z;

    std::cin >> p3.x;
    std::cin >> p3.y;
    std::cin >> p3.z;

    std::cin >> p4.x;
    std::cin >> p4.y;
    std::cin >> p4.z;

    std::cin >> p5.x;
    std::cin >> p5.y;
    std::cin >> p5.z;

    std::cin >> p6.x;
    std::cin >> p6.y;
    std::cin >> p6.z;

    plane_t pl1{p1, p2, p3}, pl2{p4, p5, p6};

    pl1.get_normal().print();
    pl2.get_normal().print();

    pl1.line_of_intersect(pl2).print();

    #endif

    return 0;
}
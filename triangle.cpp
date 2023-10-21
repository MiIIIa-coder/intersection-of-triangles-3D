#include "geom_obj.hpp"

#define TEST_INTER_PLANES 0
#define READY 0

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
        //g_obj::line_segment sec{verts[i], verts[(i+1) % 3]};
        section_arr[i] = {verts[i], verts[(i+1) % 3]};
    }
    std::vector<g_obj::line_t> line_arr(3);
    for (int i = 0; i < 3; ++i) {
        line_arr[i] = section_arr[i].get_line();
    }

    g_obj::triangle_t ret{verts, plane_, line_arr};
    return ret;
}

#if READY
bool check_tr_inter(const g_obj::triangle_t &tr1, const g_obj::triangle_t &tr2) {
    g_obj::line_t inter_line = tr1.plane.line_of_intersect(tr2.plane);
    std::vector<g_obj::point_t> inter_points1(2);
    int count_inters{0};
    for (int i = 0; i < 3 || count_inters < 2; i++) {
        if (inter_line.parallelism(tr1.lines[i]))             //if lines are parallel
            if (inter_line.point_belong(tr1.vertices[i]) &&
                inter_line.point_belong(tr1.vertices[(i+1)%3])) {
                inter_points1[0] = tr1.vertices[i]; inter_points1[1] = tr1.vertices[i];
                break;
                }
        else {   //if intersections
            g_obj::point_t inter_point = inter_line.point_of_intersect(tr1.lines[i])
            if (count_inters < 2)
        }
    }
    
    std::vector<g_obj::point_t> inter_points2(3);
}
#endif

int main(int argc, char* argv) 
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
                bool inter_tr = check_tr_inter(triangles[i], triangles[j]);
            }
        }
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
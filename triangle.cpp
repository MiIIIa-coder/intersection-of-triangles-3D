#include "geom_obj.hpp"

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
    return ret;
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
            continue;
        } else {
            for (int j = 0; j < N; j++) {
                if (i == j)
                    continue;
                if ((triangles[i].check_tr_inter(triangles[j]))) {
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

    return 0;
}
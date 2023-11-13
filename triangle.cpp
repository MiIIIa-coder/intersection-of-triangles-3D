#include "geom_obj.hpp"
#include "octree.hpp"

#define octTree 1

Singleton* Singleton::_instance = nullptr;

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

    g_obj::type_obj type;
    if (verts[0].equal(verts[1]) && verts[1].equal(verts[2])) 
        type = g_obj::POINT;
    else if ((verts[0].equal(verts[1]) && !(verts[0].equal(verts[2]))) ||
             (verts[0].equal(verts[2]) && !(verts[0].equal(verts[1]))) ||
             (verts[1].equal(verts[2]) && !(verts[1].equal(verts[0])))  ) {
                type = g_obj::SEGMENT;
             }
    else type = g_obj::TRIANGLE;

    g_obj::plane_t plane_{verts[0], verts[1], verts[2]};

    std::vector<g_obj::line_segment> section_arr(3);
    for (int i = 0; i < 3; ++i) {
        section_arr[i] = {verts[i], verts[(i+1) % 3]};
    }
    std::vector<g_obj::line_t> line_arr(3);
    for (int i = 0; i < 3; ++i) {
        line_arr[i] = section_arr[i].get_line();
    }

    g_obj::triangle_t ret{type, verts, plane_, line_arr};
    return ret;
}

int main(int argc, char** argv) 
{
    using g_obj::point_t;
    using g_obj::plane_t;
    using g_obj::triangle_t;

    int N;
    std::cin >> N;

    //using octree
    #if octTree
    float minimal {std::numeric_limits<float>::max()};
    float maximum (0);

    std::list<g_obj::triangle_t> all_triangles;
    for (int i = 0; i < N; i++) {
        triangle_t tmp_tr;
        tmp_tr = get_triangle();
        all_triangles.push_back(tmp_tr);

        if (tmp_tr.len_min_side() < minimal)
            minimal = tmp_tr.len_min_side();

        float local_max = std::max({tmp_tr.vertices[0].distance({0, 0, 0}),
                                tmp_tr.vertices[1].distance({0, 0, 0}),
                                tmp_tr.vertices[2].distance({0, 0, 0})}, comp);
        if (maximum < local_max)
            maximum = local_max;
    }

    Singleton* min_size = Singleton::Instance();
    if (min_size != nullptr) {
        min_size->Set(minimal);
        min_size->Print();
    }

    std::cout << maximum << std::endl;

    #endif

    //using O(N*N)
    #if not octTree

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
                    break;
                }
            }
        }
    }

    for (int i = 0; i < N; i++) {
        if (triangles[i].inter == true)
            std::cout << i << std::endl;
    }

    #endif    

    return 0;
}
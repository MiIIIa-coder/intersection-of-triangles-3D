#include "geom_obj.hpp"
#include "octree.hpp"

#define octTree 1

g_obj::triangle_t get_triangle(int number) {
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

    g_obj::triangle_t ret{type, verts, plane_, line_arr, false, number};
    return ret;
}

int main(int argc, char** argv) 
{
    using g_obj::point_t;
    using g_obj::vector_t;
    using g_obj::plane_t;
    using g_obj::triangle_t;

    int N;
    std::cin >> N;

    //using octree
    #if octTree
    float minimum {std::numeric_limits<float>::max()};
    float maximum (0);
    bool point_exist {false};
    bool not_point_exist {false};

    std::list<g_obj::triangle_t> all_triangles;
    for (int i = 0; i < N; i++) {
        triangle_t tmp_tr;
        tmp_tr = get_triangle(i);
        all_triangles.push_back(tmp_tr);

        float min_side = tmp_tr.len_min_side();
        if (equal_null(min_side) && !not_point_exist) 
            point_exist = true;
        else if (min_side < minimum && !equal_null(min_side)) {
            minimum = tmp_tr.len_min_side();
            not_point_exist = true;
            point_exist = false;
        }

        float local_max = std::max({tmp_tr.vertices[0].distance({0, 0, 0}),
                                    tmp_tr.vertices[1].distance({0, 0, 0}),
                                    tmp_tr.vertices[2].distance({0, 0, 0})}, comp);
        if (maximum < local_max)
            maximum = local_max;
            
    }
    if (point_exist)  //if only points
        minimum = flt_tolerance;

    //creating global space
    vector_t min_vec {-(maximum+1), -(maximum+1), -(maximum+1)};
    vector_t max_vec {  maximum+1,    maximum+1,    maximum+1 };
    octree::boundingBox global_space {min_vec, max_vec};

    //creating octree
    octree::ocTree tree {global_space, all_triangles, minimum};
    tree.build_tree();

    //here it can die
    //intersect triangles
    std::list<g_obj::triangle_t> parent_global_list;
    auto answer = tree.get_inter(parent_global_list);
    for (auto it = answer.begin(); it != answer.end(); ++it) {
        if (it->inter) std::cout << it->number << std::endl;
    }

    #endif

    //using O(N*N)
    #if not octTree

    std::vector<triangle_t> triangles(N);
    for (int i = 0; i < N; i++) {
        triangles[i] = get_triangle(i);
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
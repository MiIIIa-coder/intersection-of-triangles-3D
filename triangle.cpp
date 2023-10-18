#include "geom_obj.hpp"


int main() 
{
    using g_obj::vector_t;
    using g_obj::point_t;
    vector_t v1, v2;
    point_t  p1, p2;

    std::cin >> v1.x;
    std::cin >> v1.y;
    std::cin >> v1.z;

    std::cin >> v2.x;
    std::cin >> v2.y;
    std::cin >> v2.z;

    std::cin >> p1.x;
    std::cin >> p1.y;
    std::cin >> p1.z;

    std::cin >> p2.x;
    std::cin >> p2.y;
    std::cin >> p2.z;

    g_obj::line_t l1{p1, v1};
    g_obj::line_t l2{p2, v2};

    if (l1.parallelism(l2)) std::cout << "parallel!" << std::endl;
    else l1.point_of_intersect(l2).print();

    return 0;
}
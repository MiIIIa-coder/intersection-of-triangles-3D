#include "geom_obj.hpp"


int main() 
{
    using g_obj::vector_t;
    vector_t p1;
    vector_t p2;

    std::cin >> p1.x;
    std::cin >> p1.y;
    std::cin >> p1.z;

    std::cin >> p2.x;
    std::cin >> p2.y;
    std::cin >> p2.z;

    p1.print();

    p1.get_normal().print();

    return 0;
}
#ifndef OCTREE_
#define OCTREE_

#include <list>
#include <cstddef>

#include "geom_obj.hpp"

namespace octree
{
    enum position {IN, OUT, BORDER};
    
    struct boundingBox {
        g_obj::vector_t min;
        g_obj::vector_t max;

        boundingBox() {}
        boundingBox(g_obj::vector_t minimal, g_obj::vector_t maximum) : min(minimal), max(maximum) {}
        bool valid() const;
        g_obj::vector_t center() const;
        position point_position(g_obj::point_t &point) const;
        position triangle_position(g_obj::triangle_t &tr) const;

    };
    

    class ocTree {
    private:
        boundingBox region;
        std::list<g_obj::triangle_t> list_obj;
        std::byte m_activeNodes = (std::byte)0;
        float min_size;
    public:
        ocTree *child_node [8];
    public:
        ocTree() {}
        ocTree(boundingBox &reg, std::list<g_obj::triangle_t> &tr_list, float min) : region(reg), list_obj(tr_list), min_size(min) {}
        ~ocTree();

        void build_tree();
        std::byte get_activeNodes() const { return m_activeNodes; }
        std::list<g_obj::triangle_t> &get_list_obj() { return list_obj; }
        std::list<g_obj::triangle_t> get_inter(std::list<g_obj::triangle_t> &parent_l_obj);

    private:
        ocTree *create_node(boundingBox &region, std::list<g_obj::triangle_t> &List, float min);
        
    };
    
    
} // namespace octree




#endif
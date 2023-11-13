#ifndef OCTREE_
#define OCTREE_

#include "geom_obj.hpp"
#include <list>

//#define MAX_SIZE 128
#define MIN_SIZE 0.0125

class Singleton {
private:
    static Singleton* _instance;
    int d;

protected:
    Singleton() {
    d = 0;
    }

public:
    static Singleton* Instance() {
        if (_instance == nullptr) {
            _instance = new Singleton();
            return _instance;
        } else return nullptr;
    }

    int Get() { return d; }

    void Set(int _d) { d = _d; }

    void Print() { std::cout << "d = " << d << std::endl; }

    ~Singleton() { delete _instance; }
};

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
        position triangle_position(g_obj::triangle_t tr) const;

    };
    

    class ocTree {
    private:
        boundingBox region;
        std::list<g_obj::triangle_t> list_obj;
        std::byte m_activeNodes = (std::byte)0;
    public:
        //ocTree *child_node = new (ocTree)[8];
        ocTree *child_node [8];
        ocTree *parent;
    public:
        ocTree() {}
        ocTree(boundingBox &reg, std::list<g_obj::triangle_t> &tr_list) : region(reg), list_obj(tr_list) {}
        ~ocTree();

        void build_tree();
    private:
        ocTree *create_node(boundingBox &region, std::list<g_obj::triangle_t> &List);
        std::list<g_obj::triangle_t> get_inter(std::list<g_obj::triangle_t> &parent_l_obj);

    };
    
    
} // namespace octree




#endif
#include "octree.hpp"

namespace octree {

    //-------------------------------------
    // boundingBox
    //-------------------------------------
    bool boundingBox::valid() const{ 
        if (min.valid() && max.valid() && min.get_len() < max.get_len()) 
            return true; else return false; 
    }

    g_obj::vector_t boundingBox::center() const {
        assert(valid());

        return {(max.x - min.x) / 2,
                (max.y - min.y) / 2,
                (max.z - min.z) / 2};
    }

    position boundingBox::point_position(g_obj::point_t &point) const {
        if (abs(min.x) + flt_tolerance < abs(point.x) &&
            abs(min.y) + flt_tolerance < abs(point.y) &&
            abs(min.z) + flt_tolerance < abs(point.z) &&
            abs(max.x) - flt_tolerance > abs(point.x) &&
            abs(max.y) - flt_tolerance > abs(point.y) &&
            abs(max.z) - flt_tolerance > abs(point.z)  )
            return IN;
        else if (abs(min.x) - flt_tolerance > point.x ||
                 abs(min.y) - flt_tolerance > point.y ||
                 abs(min.z) - flt_tolerance > point.z ||
                 abs(max.x) + flt_tolerance < point.x ||
                 abs(max.y) + flt_tolerance < point.y ||
                 abs(max.z) + flt_tolerance < point.z)
            return OUT;
        else return BORDER;
    }

    position boundingBox::triangle_position(g_obj::triangle_t tr) const {
        if (point_position(tr.vertices[0]) == IN &&
            point_position(tr.vertices[1]) == IN &&
            point_position(tr.vertices[2]) == IN   )
            return IN;
        if (point_position(tr.vertices[0]) == OUT &&
            point_position(tr.vertices[1]) == OUT &&
            point_position(tr.vertices[2]) == OUT   )
            return OUT;
        
        return BORDER;
    }

    //-------------------------------------
    // ocTree
    //-------------------------------------
    ocTree *ocTree::create_node(boundingBox &region, std::list<g_obj::triangle_t> &List) {
        if (List.size() == 0)
            return nullptr;
        
        ocTree *ret = new ocTree(region, List);
        ret->parent = this;

        return ret;
    }

    void ocTree::build_tree() {
        if (list_obj.size() <= 1)
            return;
        
        g_obj::vector_t dimensions = region.max - region.min;
        if (dimensions.x < MIN_SIZE && dimensions.y < MIN_SIZE && dimensions.z < MIN_SIZE)
            return;

        g_obj::vector_t center = region.center();

        boundingBox *octant = new boundingBox[8];
        octant[0] = boundingBox(region.min, center);
        octant[1] = boundingBox({center.x, region.min.y, region.min.z}, {region.max.x, center.y, center.z    });
        octant[2] = boundingBox({center.x, region.min.y, center.z    }, {region.max.x, center.y, region.max.z});
        octant[3] = boundingBox({region.min.x, region.min.y, center.z}, {center.x, center.y, region.max.z    });
        octant[4] = boundingBox({region.min.x, region.min.y, center.z}, {center.x, center.y, region.max.z    });
        octant[5] = boundingBox({center.x, center.y, region.min.z    }, {region.max.x, region.max.y, center.z});
        octant[6] = boundingBox(center, region.max);    
        octant[7] = boundingBox({region.min.x, center.y, center.z    }, {center.x, region.max.y, region.max.z});

        std::list<g_obj::triangle_t> *octlist = new std::list<g_obj::triangle_t>[8]; //here all obj in all octants
        std::list<g_obj::triangle_t> *delist  = new std::list<g_obj::triangle_t>[1]; //here obj which were moved to low tree's level

        std::list<g_obj::triangle_t>::iterator it;
        for (it = list_obj.begin(); it != list_obj.end(); it++) {
            for (int i = 0; i < 8; i++) {
                if (octant[i].triangle_position(*it) == IN) {
                    octlist[i].push_back(*it);
                    delist[i].push_back(*it);
                    break;
                }
            }
        }

        //delete triangles which were moved to the next level
        for (it = delist->begin(); it != delist->end(); it++) {
            list_obj.remove(*it);
        }

        //creating childs
        for (int i = 0; i < 8; i++) {
            if (octlist[i].size() != 0) {
                child_node[i] = create_node(octant[i], octlist[i]);
                m_activeNodes |= (std::byte)(1 << i);
                child_node[i]->build_tree();
            }
        }
        
    }
}
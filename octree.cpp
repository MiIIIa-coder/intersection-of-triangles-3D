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
    ocTree::~ocTree() {
        for (int i = 0; i < 8; i++) {
            delete(child_node[i]);
        } 
    }

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

        boundingBox octant[8] {
            boundingBox(region.min, center),
            boundingBox({center.x, region.min.y, region.min.z}, {region.max.x, center.y, center.z    }),
            boundingBox({center.x, region.min.y, center.z    }, {region.max.x, center.y, region.max.z}),
            boundingBox({region.min.x, region.min.y, center.z}, {center.x, center.y, region.max.z    }),
            boundingBox({region.min.x, region.min.y, center.z}, {center.x, center.y, region.max.z    }),
            boundingBox({center.x, center.y, region.min.z    }, {region.max.x, region.max.y, center.z}),
            boundingBox(center, region.max),
            boundingBox({region.min.x, center.y, center.z    }, {center.x, region.max.y, region.max.z}),
        };

        std::list<g_obj::triangle_t> octlist[8]; //here all obj in all octants
        std::list<g_obj::triangle_t> delist;     //here obj which were moved to low tree's level

        std::list<g_obj::triangle_t>::iterator it;
        for (it = list_obj.begin(); it != list_obj.end(); it++) {
            for (int i = 0; i < 8; i++) {
                if (octant[i].triangle_position(*it) == IN) {
                    octlist[i].push_back(*it);
                    delist.push_back(*it);
                    break;
                }
            }
        }

        //delete triangles which were moved to the next level
        for (it = delist.begin(); it != delist.end(); it++) {
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

    std::list<g_obj::triangle_t> ocTree::get_inter(std::list<g_obj::triangle_t> &parent_l_obj) {
        std::list<g_obj::triangle_t> intersections;

        std::list<g_obj::triangle_t>::iterator it, it_local;
        for (it = parent_l_obj.begin(); it != parent_l_obj.end(); it++) {
            for (it_local = list_obj.begin(); it_local != list_obj.end(); it_local++) {
                if (it_local->check_tr_inter(*it)) {
                    if (it->inter == false) {
                        it->inter = true;
                        intersections.push_back(*it);
                    }
                    if (it_local->inter == false) {
                        it_local->inter = true;
                        intersections.push_back(*it_local);
                    }
                }
            }
        }

        //check inters between local triangles
        if (list_obj.size() > 1) {
            std::list<g_obj::triangle_t> tmp;
            tmp = list_obj;  //may be disaster
            while (tmp.size() > 0) {
                for (it = tmp.begin(); it != tmp.end(); it++) {
                    if (tmp.back() == *it) continue;
                    if (tmp.back().check_tr_inter(*it)) {
                        if (it->inter == false) {
                        it->inter = true;
                        intersections.push_back(*it);
                    }
                    if (tmp.back().inter == false) {
                        tmp.back().inter = true;
                        intersections.push_back(tmp.back());
                    }
                    }
                }
                tmp.remove(tmp.back()); 
            }
        }

        //update parent's list with new obj and move it to the next level
        for (it = list_obj.begin(); it != list_obj.end(); it++) {
            parent_l_obj.push_back(*it);
        }

        int index = 0;
        for (int flags = (int)m_activeNodes; flags > 0; flags >>= 1, index++)  //maybe problem with byte->int
            if ((flags & 1) == 1) {
                auto tmp = child_node[index]->get_inter(parent_l_obj);
                for (it = tmp.begin(); it != tmp.end(); it++)
                    intersections.push_back(*it);
            }
                
        return intersections;

    }

}
#ifndef GEOM_OBJ_H_
#define GEOM_OBJ_H_
#include <iostream>
#include <cmath>
#include <cassert>

#define flt_tolerance 0.00001
#define inter_vol_width 100

bool equal_null(float x);

namespace g_obj {

    //-------------------------------------
    // vector_t
    //-------------------------------------
    struct vector_t {

        float x = NAN, y = NAN, z = NAN;

        void print() const { std::cout << "(" << x << " ; " << y << " ; " << z << ")" << std::endl; }
        bool valid() const { return !(x != x || y!= y || z!=z); }
        bool equal(const vector_t &another) const;
        vector_t operator-(const vector_t &another) const {
            return {x - another.x, y - another.y, z - another.z};
        }
        vector_t get_normal() const;
 
    };
    

    //-------------------------------------
    // point_t
    //-------------------------------------
    struct point_t {

        float x = NAN, y = NAN, z = NAN;

        void print() const { std::cout << "(" << x << " ; " << y << " ; " << z << ")" << std::endl; }
        bool valid() const { return !(x != x || y!= y || z!=z); }
        bool equal(const point_t &another) const {
            assert(valid() && another.valid());
            return (std::abs(x - another.x) < flt_tolerance &&
                    std::abs(y - another.y) < flt_tolerance &&
                    std::abs(z - another.z) < flt_tolerance);
        }

        //this - another
        vector_t operator-(const point_t &another) const {
            return {x - another.x, y - another.y, z - another.z};
        }

    };
    

    //-------------------------------------
    // line_t (point and direction vector as point)
    //-------------------------------------
    struct  line_t {
        point_t  point_ {0.0, 0.0, 0.0};
        vector_t vec_   {1.0, 1.0, 1.0};  //point - (0;0;0) <=> vector

        line_t(const point_t point, const vector_t vec) : point_(point), vec_(vec) {}

        void print() const { point_.print(); vec_.print(); }
        bool parallelism(const line_t &another) const;
        bool point_belong(const point_t &point) const;

        //return point of intersect if it exist
        //else return non-valid point or if they are parallel
        point_t point_of_intersect(const line_t &another) const;

    };

    //-------------------------------------
    // line_segment (two points)
    //-------------------------------------
    struct line_segment {
        point_t p1_{0.0, 0.0, 0.0};
        point_t p2_{1.0, 1.0, 1.0};

        line_segment(const point_t &p1, const point_t &p2) : p1_(p1), p2_(p2)  {}

        float len() const;
        line_t get_line() const;  //where line_segment lives
    };

    //-------------------------------------
    // plane (three points)
    //-------------------------------------
    struct plane_t {
        float a = NAN, b = NAN, c = NAN, d = NAN;

        plane_t(const point_t &p1, const point_t &p2, const point_t &p3);

        bool valid() const { return !(a != a || b!= b || c!=c || d!=d); }
        vector_t get_normal() const;
        line_t line_of_intersect(const plane_t &another) const;
    };

}

#endif
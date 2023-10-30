#ifndef GEOM_OBJ_H_
#define GEOM_OBJ_H_
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>
#include <stdlib.h>

#define flt_tolerance 0.00001

bool equal_null(float x);

//A1*x + B1*y = D1
//A2*x + B2*y = D2
std::vector<float> solve_alg_sys_2(float A1, float B1, float D1,
                                   float A2, float B2, float D2);

namespace g_obj {

    enum type_obj {POINT, SEGMENT, TRIANGLE};

    //-------------------------------------
    // vector_t
    //-------------------------------------
    struct vector_t {

        float x = NAN, y = NAN, z = NAN;

        void print() const { std::cout << "(" << x << " ; " << y << " ; " << z << ")" << std::endl; }
        bool valid() const { return !(x != x || y!= y || z!=z); }
        bool equal(const vector_t &another) const;
        bool ortogonality(const vector_t &another) const;
        float get_len() const { return sqrt(x*x + y*y + z*z); }
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
        float distance(const point_t &another) const; 

        //this - another
        vector_t operator-(const point_t &another) const {
            return {x - another.x, y - another.y, z - another.z};
        }

    };
    

    //-------------------------------------
    // line_t (point and direction vector as point)
    //-------------------------------------
    struct  line_t {
        point_t  point_;
        vector_t vec_;

        line_t() {
            point_ = {0.0, 0.0, 0.0};
            vec_ = {1.0, 1.0, 1.0};
        }

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

        point_t p1_;
        point_t p2_;

        line_segment() {
            p1_ = {0.0, 0.0, 0.0};
            p2_ = {1.0, 1.0, 1.0};
        }

        line_segment(const point_t &p1, const point_t &p2) : p1_(p1), p2_(p2)  {}

        float len() const;
        line_t get_line() const;  //where line_segment lives
        bool point_belong(const point_t &point) const;
    };

    //-------------------------------------
    // plane_t (ax+by+cz+d=0)
    //-------------------------------------
    struct plane_t {
        float a, b ,c ,d;

        plane_t() {
            a = b = c = d = NAN;
        }

        plane_t(const point_t &p1, const point_t &p2, const point_t &p3);

        bool valid() const { return !(a != a || b!= b || c!=c || d!=d); }
        void print() const { if (valid()) std::cout << a << " " << b << " " << c << " " << d << std::endl;
                             else std::cout << "non-valid data of plane" << std::endl;}
        bool parallelism(const plane_t &another) const;
        bool equal_parallel(const plane_t &another) const;
        bool point_belong(const point_t &point) const;
        vector_t get_normal() const;              //normal of this plane
        vector_t get_normal(line_t &line) const;  //normal of line (normal in plane)
        point_t point_of_intersect(const line_t &line) const;
        line_t line_of_intersect(const plane_t &another) const;
    };

    //-------------------------------------
    // triangle_t (three points, lines, plane)
    //-------------------------------------
    struct triangle_t {
        type_obj type;
        std::vector<point_t> vertices;
        plane_t plane;
        std::vector<line_t> lines;
        bool inter {false};

        void print() const;
        std::vector<g_obj::point_t> find_inter_points(const g_obj::line_t &inter_line) const;
        bool check_tr_inter(const g_obj::triangle_t &tr2) const;
    };

    float det(float a, float b, float c, float d);
    float scalar_mult(const vector_t &vect1, const vector_t &vect2);
    vector_t vect_mult(const vector_t &vect1, const vector_t &vect2);
    float tr_square(point_t &p1, point_t &p2, point_t &p3);
    
}

#endif
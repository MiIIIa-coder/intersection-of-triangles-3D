#include "geom_obj.hpp"

bool equal_null(float x) {
    return std::abs(x) <= flt_tolerance;
}

namespace g_obj {

    float det(float a, float b, float c, float d) {
        return a*d - b*c;
    }

    float scalar_mult(const vector_t &vect1, const vector_t &vect2) {
        return vect1.x*vect2.x + vect1.y*vect2.y + vect1.z*vect2.z;
    }

    vector_t vect_mult(const vector_t &vect1, const vector_t &vect2) {
        vector_t ret;
        ret.x =  det(vect1.y, vect1.z, vect2.y, vect2.z);
        ret.y = -det(vect1.x, vect1.z, vect2.x, vect2.z);
        ret.z =  det(vect1.x, vect1.y, vect2.x, vect2.y);
        return ret;
    }

    //-------------------------------------
    // vector_t
    //-------------------------------------
    vector_t vector_t::get_normal() const {

        assert(valid() && (!equal_null(x) ||
                           !equal_null(y) ||
                           !equal_null(z)));

        if (!equal_null(x))
            return {-(y+z)/x, 1.0, 1.0};
        else if (!equal_null(y))
            return {1.0, -(x+z)/y, 1.0};
        else 
            return {1.0, 1.0, -(x+y)/z};
    }

    bool vector_t::equal(const vector_t &another) const {
        assert(valid() && another.valid());
        return (std::abs(x - another.x) < flt_tolerance &&
                std::abs(y - another.y) < flt_tolerance &&
                std::abs(z - another.z) < flt_tolerance);
    }

    bool vector_t::parallelism(const vector_t &another) const {
        assert(valid() && another.valid());
        return equal_null(scalar_mult(another, *this));
    }

    //-------------------------------------
    // point_t
    //-------------------------------------
    float point_t::distance(const point_t &another) const {
        return sqrt((x - another.x)*(x - another.x) +
                    (y - another.y)*(y - another.y) +
                    (z - another.z)*(z - another.z));
    }

    //-------------------------------------
    // line_segment (two points)
    //-------------------------------------
    float line_segment::len() const {
        assert(p1_.valid() && p2_.valid());
        return sqrt((p2_.x - p1_.x)*(p2_.x - p1_.x) +
                    (p2_.y - p1_.y)*(p2_.y - p1_.y) +
                    (p2_.z - p1_.z)*(p2_.z - p1_.z));
    }

    line_t line_segment::get_line() const {
        assert(p1_.valid() && p2_.valid());
        line_t ret(p1_, p2_ - p1_);
        return ret;
    }

    //-------------------------------------
    // line_t (point and direction vector)
    //-------------------------------------
    bool line_t::parallelism(const line_t &another) const {
        return equal_null(scalar_mult(vec_, another.vec_.get_normal()));
    }

    bool line_t::point_belong(const point_t &point) const {
    
    //Ox, Oy, Oz
        if (equal_null(vec_.y) && equal_null(vec_.z)) {
            if (equal_null(point.y) && equal_null(point.z))
                return true;
            else { return false; } }

        if (equal_null(vec_.x) && equal_null(vec_.z)) {
            if (equal_null(point.x) && equal_null(point.z))
                return true;
            else { return false; } }

        if (equal_null(vec_.x) && equal_null(vec_.y)) {
            if (equal_null(point.x) && equal_null(point.y))
                return true;
            else { return false; } }
    
    //in Oxy, Oxz, Oyz
        if (equal_null(vec_.z)) {
            if (equal_null(point.z) && equal_null((point.x - point_.x)/vec_.x - (point.y - point_.y)/vec_.y))
                return true;
            else { return false; } }
        
        if (equal_null(vec_.y)) {
            if (equal_null(point.y) && equal_null((point.x - point_.x)/vec_.x - (point.z - point_.z)/vec_.z))
                return true;
            else { return false; } }
        
        if (equal_null(vec_.x)) {
            if (equal_null(point.x) && equal_null((point.y - point_.y)/vec_.y - (point.z - point_.z)/vec_.z))
                return true;
            else { return false; } }
            
    //general case
        return equal_null((point.x - point_.x)/vec_.x - (point.y - point_.y)/vec_.y) &&
               equal_null((point.y - point_.y)/vec_.y - (point.z - point_.z)/vec_.z);
    }

    point_t line_t::point_of_intersect(const line_t &another) const {
        point_t ret;

        float det0   = det(vec_.x, -another.vec_.x, vec_.y, -another.vec_.y);
        float det0_1 = det(vec_.y, -another.vec_.y, vec_.z, -another.vec_.z);

        if (equal_null(det0) && equal_null(det0_1)) return ret;

        float k;
        if (!equal_null(det0)) {
            float det1 = det(another.point_.x - point_.x, -another.vec_.x,
                             another.point_.y - point_.y, -another.vec_.y);
            k = det1/det0;
        }
        else {
            float det2 = det(another.point_.y - point_.y, -another.vec_.y,
                             another.point_.z - point_.z, -another.vec_.z);
            k = det2/det0_1;
        }

        return {vec_.x*k + point_.x,
                vec_.y*k + point_.y,
                vec_.z*k + point_.z};
    }

    //-------------------------------------
    // plane_t (three points)
    //-------------------------------------
    plane_t::plane_t(const point_t &p1, const point_t &p2, const point_t &p3) {
        assert(p1.valid() && p2.valid() && p3.valid());
        a = (p1.y-p2.y)*(p2.z-p3.z) - (p2.y-p3.y)*(p1.z-p2.z);
        b = (p1.z-p2.z)*(p2.x-p3.x) - (p1.x-p2.x)*(p2.z-p3.z);
        c = (p1.x-p2.x)*(p2.y-p3.y) - (p2.x-p3.x)*(p1.y-p2.y);
        d = p1.x*((p1.z-p2.z)*(p2.y-p3.y) - (p2.z-p3.z)*(p1.y-p2.y)) +
            p1.y*((p1.x-p2.x)*(p2.z-p3.z) - (p2.x-p3.x)*(p1.z-p2.z)) +
            p1.z*((p1.y-p2.y)*(p2.x-p3.x) - (p2.y-p3.y)*(p1.x-p2.x));
    } 

    vector_t plane_t::get_normal() const {
        assert(valid());
        return {a, b, c};
    }

    vector_t plane_t::get_normal(line_t &line) const {
        assert(valid() && line.point_.valid() && line.vec_.valid());

        return vect_mult(get_normal(), line.vec_);
    }

    //if planes are parallel return non-valid line
    line_t plane_t::line_of_intersect(const plane_t &another) const {
        assert(valid() && another.valid());

        //find direction vector of line_of_intersect
        vector_t inter_v;
        inter_v = vect_mult(get_normal(), another.get_normal());
        if (inter_v.equal({0, 0, 0})) {
            point_t point;
            vector_t vector;
            return {point, vector};
        }

        //find point inter_p in line_of_intersect (z = 0)
        float det0 = det(a, b, another.a, another.b);
        float det1 = det(-d, b, -another.d, another.b);
        float det2 = det(a, -d, another.a, -another.d);
        point_t inter_p;
        inter_p.x = det1/det0; inter_p.y = det2/det0; inter_p.z = 0;

        return {inter_p, inter_v};
    }

    //-------------------------------------
    // triangle_t (three points)
    //-------------------------------------
    void triangle_t::print() const {
        for (size_t i = 0; i < vertices.size(); ++i) {
            vertices[i].print();
        }

        std::cout << "plane: " << std::endl;
        plane.print();
        std::cout << "normal vector: " << std::endl;
        plane.get_normal().print();

        std::cout << "lines on triangle's sides" << std::endl;
        for (size_t i = 0; i < lines.size(); ++i) {
            lines[i].print();
        }

        if (inter)
            std::cout << "true" << std::endl;
        else 
            std::cout << "false" << std::endl;
    }

} // namespace g_obj

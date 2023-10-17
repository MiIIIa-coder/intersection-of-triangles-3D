#include "geom_obj.hpp"

bool equal_null(float x) {
    return std::abs(x) <= flt_tolerance;
}

namespace g_obj {

    float scalar_mult(const vector_t &vect1, const vector_t vect2) {
        return vect1.x*vect2.x + vect1.y*vect2.y + vect1.z*vect2.z;
    }

    float det(float a, float b, float c, float d) {
        return a*c - b*d;
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
    // line_t (point and direction vector as point)
    //-------------------------------------
    bool line_t::parallelism(const line_t &another) const {
        return equal_null(scalar_mult(vec_, another.vec_.get_normal()));
    }

    bool line_t::point_belong(const point_t &point) const {
        return equal_null((point.x - point_.x)/vec_.x - (point.y - point_.y)/vec_.y) &&
               equal_null((point.y - point_.y)/vec_.y - (point.z - point_.z)/vec_.z);
    }

    point_t line_t::point_of_intersect(const line_t &another) const {
        point_t ret;

        if (parallelism(another)) return ret;

        float det0 = det(vec_.x, -another.vec_.x, vec_.y, -another.vec_.y);
        float det1 = det(another.point_.x - point_.x, -another.vec_.x,
                         another.point_.y - point_.y, -another.vec_.y);
        float k = det1/det0;

        return {vec_.x*k + point_.x,
                vec_.y*k + point_.y,
                vec_.z*k + point_.z};
    }

} // namespace g_obj

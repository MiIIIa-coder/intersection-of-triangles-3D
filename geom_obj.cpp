#include "geom_obj.hpp"

bool equal_null(float x) {
    return std::abs(x) <= flt_tolerance;
}

//A1*x + B1*y = D1
//A2*x + B2*y = D2
std::vector<float> solve_alg_sys_2(float A1, float B1, float D1,
                                   float A2, float B2, float D2) {
    std::vector<float> ans(2);
    float det0 = g_obj::det(A1, B1, A2, B2);
    assert(!equal_null(det0));
    float det1 = g_obj::det(D1, B1, D2, B2);
    float det2 = g_obj::det(A1, D1, A2, D2);
    ans[0] = det1/det0;
    ans[1] = det2/det0;

    return ans;
}

bool comp(float x, float y) {
    return (x < y);
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

    float tr_square(const point_t &p1, const point_t &p2, const point_t &p3) {
        return vect_mult(p2-p1, p3-p1).get_len() * 0.5;
    }

    //check for belonging point to segment of inter_points
    bool check_point_belongs(const std::vector<g_obj::point_t> &inter_points, const g_obj::point_t &point) {
        if (inter_points.size() == 1) {
            std::cerr << "ERROR: incorrect number of points of intersection" << std::endl;
            exit(EXIT_FAILURE);
        }

        g_obj::line_segment section{inter_points[0], inter_points[1]};
        float len_section = section.len();

        if (point.distance(inter_points[0]) <= len_section &&
            point.distance(inter_points[1]) <= len_section)
            return true;
        return false;
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

    bool vector_t::ortogonality(const vector_t &another) const {
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

    bool point_t::operator==(const point_t &point) const { 
            if (equal_null(x - point.x) &&
                equal_null(y - point.y) &&
                equal_null(z - point.z)  )
                return true;
            return false;
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

    bool line_segment::point_belong(const point_t &point) const {
        int length;
        line_t line;

        length = len();
        line = get_line();
        if (line.point_belong(point)) {
            if ((length - point.distance(p1_) >= flt_tolerance) &&
                (length - point.distance(p2_) >= flt_tolerance))
                    return true;
        }

        return false;
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

    //return non-valid point if laines are parallel
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

    bool plane_t::parallelism(const plane_t &another) const {
        assert(valid() && another.valid());
        return vect_mult(get_normal(), another.get_normal()).equal({0, 0, 0});
    }

    //return true if parallel planes are equal
    bool plane_t::equal_parallel(const plane_t &another) const {
        assert(valid() && another.valid());
        if (parallelism(another) && d!=0) {
            if (!equal_null(a)) {
                if (equal_null(another.a/a - another.d/d)) return true; else return false; }
            else if (!equal_null(b)) {
                if (equal_null(another.b/b - another.d/d)) return true; else return false; }
            else if (!equal_null(c)) {
                if (equal_null(another.c/c - another.d/d)) return true; else return false; }
        } else if (equal_null(d) && equal_null(another.d)) return true;

        return false;
    }

    bool plane_t::point_belong(const point_t &point) const {
        return equal_null(a*point.x + b*point.y + c*point.z + d);
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
        std::vector<float> ans_sys(2);
        point_t inter_p;

        if (!equal_null(det(a, b, another.a, another.b))) { //z = 0 
            ans_sys = solve_alg_sys_2(a, b, -d, another.a, another.b, -another.d);
            inter_p.x = ans_sys[0]; inter_p.y = ans_sys[1]; inter_p.z = 0;
        } else if (!equal_null(det(a, c, another.a, another.c))) { //y = 0
            ans_sys = solve_alg_sys_2(a, c, -d, another.a, another.c, -another.d);
            inter_p.x = ans_sys[0]; inter_p.y = 0; inter_p.z = ans_sys[1];
        } else if (!equal_null(det(b, c, another.b, another.c))) { //x = 0
            ans_sys = solve_alg_sys_2(b, c, -d, another.b, another.c, -another.d);
            inter_p.x = 0; inter_p.y = ans_sys[0]; inter_p.z = ans_sys[1];
        } 

        return {inter_p, inter_v};
    }

    //if line and plane are parallel return non-valid
    point_t plane_t::point_of_intersect(const line_t &line) const {
        assert(line.vec_.valid() && line.point_.valid());
        point_t point;
        float scalar_mult_res;
        scalar_mult_res = scalar_mult(get_normal(), line.vec_);
        if (equal_null(scalar_mult_res)) {
            return point;
        }
        float k = (a*line.point_.x + b*line.point_.y + c*line.point_.z + d)/scalar_mult_res;
        return {line.point_.x - line.vec_.x*k,
                line.point_.y - line.vec_.y*k,
                line.point_.z - line.vec_.z*k};
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

    float triangle_t::square() const {
        return vect_mult(vertices[1] - vertices[0],
                         vertices[2] - vertices[0]).get_len() * 0.5;
    }

    float triangle_t::len_min_side() const {
        return std::min({vertices[0].distance(vertices[1]),
                         vertices[1].distance(vertices[2]),
                         vertices[2].distance(vertices[0])}, comp);
    }

    std::vector<g_obj::point_t> triangle_t::find_inter_points(const g_obj::line_t &inter_line) const {

        std::vector<g_obj::point_t> inter_points(2);
        int count_inters{0};

        for (int i = 0; i < 3 && count_inters < 2; i++) {
            g_obj::line_t line = lines[i];
            if (plane.get_normal(line).ortogonality(inter_line.vec_)) { //check for parallel between side and inter_line
                if (inter_line.point_belong(vertices[i]) &&
                    inter_line.point_belong(vertices[(i+1)%3])) {
                    inter_points[0] = vertices[i]; inter_points[1] = vertices[(i+1)%3];
                    count_inters = 2;
                    break;
                }
            }
            else {   //if intersections
                g_obj::point_t intersec_point = inter_line.point_of_intersect(lines[i]);
                if (check_point_belongs({vertices[i], vertices[(i+1)%3]}, intersec_point)) {
                    inter_points[count_inters] = intersec_point;
                    count_inters++;
                }
                if (count_inters == 2) break;
            }
        }

        if (inter_points.size() == 1) {
            std::cerr << "ERROR: incorrect number of points of intersection" << std::endl;
            exit(EXIT_FAILURE);
        }

        return inter_points;
    }

    bool triangle_t::point_in_tr(const point_t &point) const {
        float s0, s;

        if (!(plane.point_belong(point)))
            return false;
        s = 0;
        s0 = square();
        for (int i = 0; i < 3; i++) {
            s += tr_square(point, vertices[i], vertices[(i+1)%3]);
        }
        if (equal_null(s0 - s))
            return true;
        return false;
    }

    bool triangle_t::tr_in_tr(const triangle_t &triangle) const {
        if (point_in_tr(triangle.vertices[0]) &&
            point_in_tr(triangle.vertices[1]) &&
            point_in_tr(triangle.vertices[2])  )
            return true; 
        else if (triangle.point_in_tr(vertices[0]) &&
            triangle.point_in_tr(vertices[1]) &&
            triangle.point_in_tr(vertices[2])  )
            return true;

        return false;
    }

    bool triangle_t::operator==(const triangle_t &triangle) const {
        for (int i = 0; i < 3; i++) {
            if (vertices[(0 + i)%3] == triangle.vertices[0] &&
                vertices[(1 + i)%3] == triangle.vertices[1] &&
                vertices[(2 + i)%3] == triangle.vertices[2] &&
                number == triangle.number)
                return true;
        }

        return false;
    }

    bool triangle_t::check_tr_inter(const g_obj::triangle_t &tr2) const {

        //triangles are 2 points
        if (type == POINT && tr2.type == POINT) {
            if (vertices[0].equal(tr2.vertices[0]))
                return true;
            else return false;
        }

        //triangles are 2 segments
        if (type == SEGMENT && tr2.type == SEGMENT) {
            line_t line1, line2;
            point_t p1, p2, tr2_p1, tr2_p2;            //p1 != p2 
            line_segment segment1, segment2;
            for (int i = 0; i < 3; i++) {
                if (lines[i].vec_.equal({0, 0, 0})) {  //point[i] == point[i+1]
                    line1 = lines[(i+1)%3];
                    p1 = vertices[(i+1)%3];
                    p2 = vertices[(i+2)%3];
                    segment1 = {p1, p2};
                }
            }
            for (int i = 0; i < 3; i++) {
                if (tr2.lines[i].vec_.equal({0, 0, 0})) {  //point[i] == point[i+1]
                    line2 = tr2.lines[(i+1)%3];
                    tr2_p1 = tr2.vertices[(i+1)%3];
                    tr2_p2 = tr2.vertices[(i+2)%3];
                    segment2 = {tr2_p1, tr2_p2};
                }
            }

            point_t point_inter;
            int len_s1, len_s2;
            len_s1 = segment1.len();
            len_s2 = segment2.len();
            point_inter = line1.point_of_intersect(line2);
            if ((len_s1 - point_inter.distance(p1) >= flt_tolerance) &&
                (len_s1 - point_inter.distance(p2) >= flt_tolerance) &&
                (len_s2 - point_inter.distance(tr2_p1) >= flt_tolerance) &&
                (len_s2 - point_inter.distance(tr2_p2) >= flt_tolerance))
                    return true;
            else return false;

        }

        //triangles are point and segment
        if (type == POINT && tr2.type == SEGMENT) {
            point_t tr2_p1, tr2_p2;
            line_segment segment2;
            for (int i = 0; i < 3; i++) {
                if (tr2.lines[i].vec_.equal({0, 0, 0})) {  //point[i] == point[i+1]
                    tr2_p1 = tr2.vertices[(i+1)%3];
                    tr2_p2 = tr2.vertices[(i+2)%3];
                    segment2 = {tr2_p1, tr2_p2};
                }
            }
            if (segment2.point_belong(vertices[0]))
                return true;
            return false;
        } else if (type == SEGMENT && tr2.type == POINT) {
            point_t p1, p2;
            line_segment segment1;
            for (int i = 0; i < 3; i++) {
                if (lines[i].vec_.equal({0, 0, 0})) {
                    p1 = vertices[(i+1)%3];
                    p2 = vertices[(i+2)%3];
                    segment1 = {p1, p2};
                }
            }
            if (segment1.point_belong(tr2.vertices[0]))
                return true;
            return false;
        }

        //triangle and point
        if (type == TRIANGLE && tr2.type == POINT) {
            if (point_in_tr(tr2.vertices[0])) 
                return true; else return false;
        } else if (type == POINT && tr2.type == TRIANGLE) {
            if (tr2.point_in_tr(vertices[0])) 
                return true; else return false;
        }

        //triangle and segment
        if (type == TRIANGLE && tr2.type == SEGMENT) {
            if (point_in_tr(tr2.vertices[0]) &&
                point_in_tr(tr2.vertices[1]) &&
                point_in_tr(tr2.vertices[2])   )
                return true; else return false;

            line_t line;
            point_t p1, p2, point;
            line_segment segment1;
            for (int i = 0; i < 3; i++) {
                if (lines[i].vec_.equal({0, 0, 0})) {
                    p1 = vertices[(i+1)%3];
                    p2 = vertices[(i+2)%3];
                    segment1 = {p1, p2};
                }
            }
            line = segment1.get_line();
            point = plane.point_of_intersect(line);
            if (!(point.valid()))
                return false;
            if (!(segment1.point_belong(point)))
                return false;
            float s0, s;

            s = 0;
            s0 = tr_square(vertices[0], vertices[1], vertices[2]);
            for (int i = 0; i < 3; i++) {
                s += tr_square(point, vertices[i], vertices[(i+1)%3]);
            }
            if (equal_null(s0 - s))
                return true;
            return false;
        } else if (type == SEGMENT && tr2.type == TRIANGLE) {
            if (tr2.point_in_tr(vertices[0]) &&
                tr2.point_in_tr(vertices[1]) &&
                tr2.point_in_tr(vertices[2])   )
                return true; else return false;

            line_t line;
            point_t tr2_p1, tr2_p2, point;
            line_segment segment2;
            for (int i = 0; i < 3; i++) {
                if (tr2.lines[i].vec_.equal({0, 0, 0})) {
                    tr2_p1 = tr2.vertices[(i+1)%3];
                    tr2_p2 = tr2.vertices[(i+2)%3];
                    segment2 = {tr2_p1, tr2_p2};
                }
            }
            line = segment2.get_line();
            point = tr2.plane.point_of_intersect(line);
            if (!(point.valid()))
                return false;
            if (!(segment2.point_belong(point)))
                return false;
            float s0, s;

            s = 0;
            s0 = tr_square(tr2.vertices[0], tr2.vertices[1], tr2.vertices[2]);
            for (int i = 0; i < 3; i++) {
                s += tr_square(point, tr2.vertices[i], tr2.vertices[(i+1)%3]);
            }
            if (equal_null(s0 - s))
                return true;
            return false;
        }

        //2 triangles
        //here checking for parallelism of planes...
        if (plane.parallelism(tr2.plane)) {
            if (plane.equal_parallel(tr2.plane)) {  //if planes are same

                if (tr_in_tr(tr2)) return true; //if triangle in another triangle


                for (int i = 0; i < 3; i++) {
                    line_t line_side = lines[i];
                    point_t p_inter;
                    for (int j = 0; j < 3; j++) {
                        p_inter = line_side.point_of_intersect(tr2.lines[j]);
                        if (!p_inter.valid())
                            continue;
                        if (check_point_belongs({vertices[i], vertices[(i+1)%3]}, p_inter))
                            return true;
                    }
                }
                return false;
            } else return false; //different parallel planes
        }

        g_obj::line_t inter_line = plane.line_of_intersect(tr2.plane);
        
        std::vector<g_obj::point_t> inter_points1(2), inter_points2(2);

        inter_points1 = find_inter_points(inter_line); //0 or 2 points
        inter_points2 = tr2.find_inter_points(inter_line); //0 or 2 points

        if (!(inter_points1[0].valid()) || !(inter_points2[0].valid()))
            return false;

        if (inter_points1[0].equal(inter_points1[1])) {
            if (inter_points2[0].equal(inter_points2[1])) {
                if (inter_points1[0].equal(inter_points2[0]))
                    return true;
                else return false;
            }
            else if (check_point_belongs(inter_points2, inter_points1[0]))
                return true;
            else return false;
        }

        for (int i = 0; i < 2; i++) {
            if (check_point_belongs(inter_points1, inter_points2[i]))
                return true;
            //else return false;        
        }

        return false;
    }

} // namespace g_obj


#include <Eigen/Dense>
#include <variant>

namespace intersection {

struct point {
    Eigen::Vector3f pos;
};
struct line {
    point pos;
    Eigen::Vector3f dir;
};
struct ray {
    point pos;
    Eigen::Vector3f dir;
};
struct segment {
    point pos;
    Eigen::Vector3f dir;
};
struct plane {
    Eigen::Vector3f pos;
};
struct sphere {
    Eigen::Vector3f pos;
    float radius;
};
struct capsule {
    segment axis;
    float radius;
};

float distance(line line, point point) {
    return (line.pos - point.pos - line.dir * line.dir.dot(line.pos - point.pos)).norm();
}

float squared_distance(line a, line b) {
    return (b.pos - a.pos).dot(b.dir.cross(a.dir).normalized()).norm_squared();
}
float squared_distance(line a, line b) {
    //TODO
    return std::monostate();
}

float distance(plane plane, point point) {
    return point.pos.dot(plane.pos.normalized()) - plane.pos;
}

}

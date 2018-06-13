#include <Eigen/Dense>
#include <variant>
#include <cmath>

struct point {
    Eigen::Vector3f pos;
};
struct line {
    Eigen::Vector3f pos, dir;
};
struct plane {
    Eigen::Vector3f dir;
    float pos;
};
struct sphere {
    Eigen::Vector3f pos;
    float radius;
};
struct circle {
    plane plane;
    float radius;
};
struct capsule {
    line axis;
    float radius;
};

float distance(line line, point point) {
    return (line.pos - point.pos - line.dir * line.dir.dot(line.pos - point.pos)).norm();
}

float distance(line a, line b) {
    return (b.pos - a.pos).dot(b.dir.cross(a.dir).normalized());
}
std::variant<std::monostate, point, line> intersection(line a, line b) {
    //TODO
    return std::monostate();
}

float distance(plane plane, point point) {
    return point.pos.dot(plane.dir.normalized()) - plane.pos;
}

std::variant<float, point> intersection_or_distance(plane plane, line line) {
    if (line.dir.dot(plane.dir) == 0) {
        return plane.pos - line.pos.dot(plane.dir.normalized());
    } else {
        float d = (plane.pos - line.pos.dot(plane.dir)) / line.dir.dot(plane.dir);
        return point{d * line.dir + line.pos};
    }
}

std::variant<std::monostate, point, circle> sphere_plane(sphere sphere, plane plane) {
    float d = std::abs(plane.pos - sphere.pos.dot(plane.dir.normalized()));
    if (d > sphere.radius) {
        return std::monostate();
    } else if (d < sphere.radius) {
        //TODO
    } else {
        //TODO
    }
    return std::monostate();
}

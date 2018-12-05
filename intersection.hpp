#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace intersection {

    using point = Eigen::Vector3f;
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
    struct sphere {
        point pos;
        float radius;
    };
    struct capsule {
        segment segment;
        float radius;
    };

    //distances between 0d things
    float squared_distance(point point0, point point1) {
        return (point0 - point1).squaredNorm();
    }

    //distances between 1d and 0d things
    template <typename T>
    float squared_distance_impl(point point, T x) {
        float t = (point - x.pos).dot(x.dir) / x.dir.squaredNorm();
        if constexpr (std::is_same<T, ray>::value || std::is_same<T, segment>::value)
            t = std::max(t, 0.0f);
        if constexpr (std::is_same<T, segment>::value)
            t = std::min(t, 1.0f);
        return (x.pos + t * x.dir - point).squaredNorm();
    }
    float squared_distance(point point, line line) {
        return squared_distance_impl(point, line);
    }
    float squared_distance(point point, ray ray) {
        return squared_distance_impl(point, ray);
    }
    float squared_distance(point point, segment segment) {
        return squared_distance_impl(point, segment);
    }

    //distances between 1d things
    template <typename T0, typename T1>
    float squared_distance_impl(T0 a, T1 b) {
        Eigen::Vector3f n1 = a.dir.cross(b.dir.cross(a.dir));
        float t0 = (a.pos - b.pos).dot(n1) / b.dir.dot(n1);
        if constexpr (std::is_same<T0, ray>::value || std::is_same<T0, segment>::value)
            t0 = std::max(t0, 0.0f);
        if constexpr (std::is_same<T0, segment>::value)
            t0 = std::min(t0, 1.0f);
        Eigen::Vector3f p0 = b.pos + t0 * b.dir;
        Eigen::Vector3f n0 = b.dir.cross(a.dir.cross(b.dir));
        float t1 = (b.pos - a.pos).dot(n0) / a.dir.dot(n0);
        if constexpr (std::is_same<T1, ray>::value || std::is_same<T1, segment>::value)
            t1 = std::max(t1, 0.0f);
        if constexpr (std::is_same<T1, segment>::value)
            t1 = std::min(t1, 1.0f);
        Eigen::Vector3f p1 = b.pos + t1 * b.dir;
        return (p1 - p0).squaredNorm();
    }
    float squared_distance(line line0, line line1) {
        return squared_distance_impl(line0, line1);
    }
    float squared_distance(line line, ray ray) {
        return squared_distance_impl(line, ray);
    }
    float squared_distance(line line, segment segment) {
        return squared_distance_impl(line, segment);
    }
    float squared_distance(ray ray0, ray ray1) {
        return squared_distance_impl(ray0, ray1);
    }
    float squared_distance(ray ray, segment segment) {
        return squared_distance_impl(ray, segment);
    }
    float squared_distance(segment segment0, segment segment1) {
        return squared_distance_impl(segment0, segment1);
    }

    //distances between 3d and 0d things
    float distance(point point, sphere sphere) {
        return std::sqrt(squared_distance(point, sphere.pos)) - sphere.radius;
    }
    float distance(point point, capsule capsule) {
        return std::sqrt(squared_distance(point, capsule.segment)) - capsule.radius;
    }

    //distances between 3d and 1d things
    float distance(sphere sphere, line line) {
        return std::sqrt(squared_distance(sphere.pos, line)) - sphere.radius;
    }
    float distance(sphere sphere, ray ray) {
        return std::sqrt(squared_distance(sphere.pos, ray)) - sphere.radius;
    }
    float distance(sphere sphere, segment segment) {
        return std::sqrt(squared_distance(sphere.pos, segment)) - sphere.radius;
    }
    float distance(capsule capsule, line line) {
        return std::sqrt(squared_distance(line, capsule.segment)) - capsule.radius;
    }
    float distance(capsule capsule, ray ray) {
        return std::sqrt(squared_distance(ray, capsule.segment)) - capsule.radius;
    }
    float distance(capsule capsule, segment segment) {
        return std::sqrt(squared_distance(segment, capsule.segment)) - capsule.radius;
    }

    //distances between 3d and 3d things
    float distance(sphere sphere0, sphere sphere1) {
        return std::sqrt(squared_distance(sphere0.pos, sphere1.pos)) - sphere0.radius - sphere1.radius;
    }
    float distance(sphere sphere, capsule capsule) {
        return std::sqrt(squared_distance(sphere.pos, capsule.segment)) - sphere.radius - capsule.radius;
    }
    float distance(capsule capsule0, capsule capsule1) {
        return std::sqrt(squared_distance(capsule0.segment, capsule1.segment)) - capsule0.radius - capsule1.radius;
    }

    //intersections
    template <typename T0, typename T1>
    bool intersects(T0 a, T1 b) {
        constexpr bool sqrt_needed =
            std::is_same<T0, sphere>::value ||
            std::is_same<T0, capsule>::value ||
            std::is_same<T1, sphere>::value ||
            std::is_same<T1, capsule>::value;
        if constexpr (sqrt_needed) {
            return distance(a, b) < 0;
        } else {
            return squared_distance(a, b) < 0;
        }
    }
}

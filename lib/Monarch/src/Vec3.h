#ifndef MONARCH_VEC3_H
#define MONARCH_VEC3_H

#include <math.h>

// Plain 3-vector. Kept free of Arduino.h so the logic in this library can be
// compiled and unit-tested on a PC.
struct Vec3 {
    float x;
    float y;
    float z;

    float &at(int i) { return i == 0 ? x : (i == 1 ? y : z); }
    float at(int i) const { return i == 0 ? x : (i == 1 ? y : z); }

    float norm() const { return sqrtf(x * x + y * y + z * z); }
};

inline Vec3 operator+(const Vec3 &a, const Vec3 &b) { return Vec3{a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3 &a, const Vec3 &b) { return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3 &a, float s) { return Vec3{a.x * s, a.y * s, a.z * s}; }

inline float dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
    return Vec3{a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
}

#endif // MONARCH_VEC3_H

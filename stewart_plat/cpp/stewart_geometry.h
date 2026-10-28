#pragma once
#include <Arduino.h>
#include <math.h>

struct Vec3 {
    float x, y, z;
    Vec3(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
    Vec3 operator + (const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator - (const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator * (float s)     const { return Vec3(x * s, y * s, z * s); }
};

struct Pose6 {
    float x, y, z, roll, pitch, yaw; // roll/pitch/yaw: radians
    Pose6(float x_ = 0, float y_ = 0, float z_ = 0,
          float r_ = 0, float p_ = 0, float y_ = 0)
        : x(x_), y(y_), z(z_), roll(r_), pitch(p_), yaw(y_) {}

    Pose6 operator + (const Pose6& o) const {
        return Pose6(x + o.x, y + o.y, z + o.z,
                     roll + o.roll, pitch + o.pitch, yaw + o.yaw);
    }
    Pose6 operator - (const Pose6& o) const {
        return Pose6(x - o.x, y - o.y, z - o.z,
                     roll - o.roll, pitch - o.pitch, yaw - o.yaw);
    }
    Pose6 operator * (float s) const {
        return Pose6(x * s, y * s, z * s,
                     roll * s, pitch * s, yaw * s);
    }
};

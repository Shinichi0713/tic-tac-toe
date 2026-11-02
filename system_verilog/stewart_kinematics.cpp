#include "stewart_kinematics.h"

StewartKinematics::StewartKinematics() {
    setGeometry(100.0f, 60.0f, 15.0f, 10.0f);
}

void StewartKinematics::setGeometry(float baseR, float platR,
                                      float baseHalfAngDeg, float platHalfAngDeg) {
    buildJoints(baseR, baseHalfAngDeg * PI / 180.0f, baseJoints, true);
    buildJoints(platR,  platHalfAngDeg * PI / 180.0f, platJointsHome, false);
    for (int i = 0; i < 6; ++i) {
        Vec3 d = platJointsHome[i] - baseJoints[i];
        homeLegLengths[i] = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
    }
}

void StewartKinematics::buildJoints(float radius, float halfAngRad, Vec3 out[6], bool isBase) {
    for (int i = 0; i < 6; ++i) {
        float ang = i * (PI / 3.0f);
        float offset = ((i % 2 == 0) ? -1.0f : 1.0f) * halfAngRad;
        if (!isBase) offset = -offset;
        float theta = ang + offset;
        out[i] = Vec3(radius * cosf(theta), radius * sinf(theta), 0.0f);
    }
}

void StewartKinematics::rotationMatrix(float r, float p, float yaw, float R[3][3]) const {
    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(yaw), sy = sinf(yaw);
    // Rz * Ry * Rx
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

bool StewartKinematics::computeIK(const Pose6& pose, float legLengths[6]) const {
    float R[3][3];
    rotationMatrix(pose.roll, pose.pitch, pose.yaw, R);
    bool ok = true;
    for (int i = 0; i < 6; ++i) {
        Vec3 p = platJointsHome[i];
        Vec3 rot(
            R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z,
            R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z,
            R[2][0] * p.x + R[2][1] * p.y + R[2][2] * p.z
        );
        Vec3 world = Vec3(pose.x, pose.y, pose.z) + rot;
        Vec3 d = world - baseJoints[i];
        legLengths[i] = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
        if (legLengths[i] < 50.0f || legLengths[i] > 250.0f) ok = false;
    }
    return ok;
}

void StewartKinematics::getHomeLegLengths(float lengths[6]) const {
    for (int i = 0; i < 6; ++i) lengths[i] = homeLegLengths[i];
}

float StewartKinematics::checkCondition(const Pose6& pose) const {
    float R[3][3];
    rotationMatrix(pose.roll, pose.pitch, pose.yaw, R);
    float minAngle = PI / 2.0f;
    for (int i = 0; i < 6; ++i) {
        Vec3 p = platJointsHome[i];
        Vec3 rot(
            R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z,
            R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z,
            R[2][0] * p.x + R[2][1] * p.y + R[2][2] * p.z
        );
        Vec3 world = Vec3(pose.x, pose.y, pose.z) + rot;
        Vec3 d = world - baseJoints[i];
        float len = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
        if (len > 0.0f) {
            float horiz = sqrtf(d.x * d.x + d.y * d.y);
            float angle = asinf(horiz / len); // angle from vertical
            if (angle < minAngle) minAngle = angle;
        }
    }
    return minAngle / (PI / 2.0f); // 1.0 = vertical/healthy
}

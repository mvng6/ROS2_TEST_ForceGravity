#pragma once

#include <cmath>
#include <string>

struct Vector3d {
    double x = 0.0, y = 0.0, z = 0.0;
    Vector3d() = default;
    Vector3d(double x, double y, double z) : x(x), y(y), z(z) {}
};
class Tool {
public:
    double mass;
    Vector3d centerOfMass;
    Tool() : mass(0.0), centerOfMass(0, 0, 0) {}
    Tool(double m, const Vector3d& com) : mass(m), centerOfMass(com) {}
};
struct RotationMatrix {
    double m[3][3];
    RotationMatrix() {
        m[0][0] = 1.0; m[0][1] = 0.0; m[0][2] = 0.0;
        m[1][0] = 0.0; m[1][1] = 1.0; m[1][2] = 0.0;
        m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0;
    }
};
struct Wrench {
    double Fx = 0.0, Fy = 0.0, Fz = 0.0;
    double Tx = 0.0, Ty = 0.0, Tz = 0.0;
};


// -- 수학 함수들 --
// G_sensor = R_transpose * G_world 계산을 수행
inline Vector3d MatrixVectorMultiply(const RotationMatrix& R, const Vector3d& v) {
    Vector3d result;
    // R의 행이 아닌 열을 사용하여 곱셈 (Transpose 효과)
    result.x = R.m[0][0] * v.x + R.m[1][0] * v.y + R.m[2][0] * v.z;
    result.y = R.m[0][1] * v.x + R.m[1][1] * v.y + R.m[2][1] * v.z;
    result.z = R.m[0][2] * v.x + R.m[1][2] * v.y + R.m[2][2] * v.z;
    return result;
}

inline Vector3d VectorCrossProduct(const Vector3d& a, const Vector3d& b) {
    Vector3d result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

inline Wrench calculateGravityWrench(const RotationMatrix& R, const Tool& tool) {
    const double g = 9.80665;
    Vector3d G_world(0.0, 0.0, -tool.mass * g);
    Vector3d G_sensor = MatrixVectorMultiply(R, G_world);
    Vector3d F_gravity(-G_sensor.x, -G_sensor.y, -G_sensor.z);
    Vector3d T_gravity = VectorCrossProduct(tool.centerOfMass, F_gravity);
    Wrench result;
    result.Fx = F_gravity.x; result.Fy = F_gravity.y; result.Fz = F_gravity.z;
    result.Tx = T_gravity.x; result.Ty = T_gravity.y; result.Tz = T_gravity.z;
    return result;
}

// 로깅 및 디버깅용 도우미 함수
inline std::string WrenchToString(const Wrench& w) {
    return "Fx: " + std::to_string(w.Fx) + ", Fy: " + std::to_string(w.Fy) + ", Fz: " + std::to_string(w.Fz) +
           ", Tx: " + std::to_string(w.Tx) + ", Ty: " + std::to_string(w.Ty) + ", Tz: " + std::to_string(w.Tz);
}
inline std::string RotationMatrixToString(const RotationMatrix& R) {
    char buffer[200];
    snprintf(buffer, sizeof(buffer), "\n| % .3f, % .3f, % .3f |\n| % .3f, % .3f, % .3f |\n| % .3f, % .3f, % .3f |",
        R.m[0][0], R.m[0][1], R.m[0][2],
        R.m[1][0], R.m[1][1], R.m[1][2],
        R.m[2][0], R.m[2][1], R.m[2][2]);
    return std::string(buffer);
}

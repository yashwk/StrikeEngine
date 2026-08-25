#pragma once

#include <cmath>

struct Vec3 { float x; float y; float z; };
struct Mat4 { float m[4][4]; };

struct EditorCamera {
	float yaw = 0.78f;    // 45 deg
	float pitch = 0.5f;   // 30 deg
	float distance = 5.0f;
	Vec3 target = {0.0f, 0.0f, 0.0f};
	Vec3 position = {0.0f, 0.0f, 0.0f}; // Calculated
};

class ViewportPanel {
public:
	static void draw(bool* p_open = nullptr);

private:
	static EditorCamera camera;

	// 3D Math Helpers
	static Mat4 Perspective(float fov, float aspect, float near, float far);
	static Mat4 LookAt(Vec3 eye, Vec3 center, Vec3 up);
	static Mat4 Multiply(Mat4 a, Mat4 b);
	static Vec3 Project(Vec3 p, Mat4 viewProj, float w, float h);
};
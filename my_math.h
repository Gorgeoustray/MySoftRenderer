#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <initializer_list>
#include <algorithm>

using std::ostream;
using std::vector;
using std::cout;
using std::endl;

const double PI = acos(-1);

// 绝对值
template <class T>
inline T abs(T num) {
	return num > 0.0 ? num : -num;
}

// 插值
template <class T>
inline T interp(T x, T y, const double k) {
	return x + k * (y - x);
}

// 限定范围
template <class T>
inline T clamp(T val, T min, T max) {
	return val < min ? min : (val > max ? max : val);
}

// 前置声明
class Matrix;
class Vector3;
typedef Vector3 Color;

// 三维向量
// 1. 用于表示颜色
// 2. 用于不参与齐次坐标运算的方向
class Vector3 {
public:
	double x;
	double y;
	double z;

	Vector3() : x(0.0), y(0.0), z(0.0) {}
	Vector3(const double ix, const double iy, const double iz)
		: x(ix), y(iy), z(iz) {}

public:
	Vector3 operator - () const {
		return Vector3(-x, -y, -z);
	}

	Vector3 operator + (const Vector3& rhs) const {
		return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	Vector3 operator - (const Vector3& rhs) const {
		return Vector3(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	void operator += (const Vector3& rhs) {
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
	}

	void operator -= (const Vector3& rhs) {
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
	}

	void operator *= (const double rhs) {
		x *= rhs;
		y *= rhs;
		z *= rhs;
	}

	void operator /= (const double rhs) {
		if (rhs) {
			double rRhs = 1.0 / rhs;
			x *= rRhs;
			y *= rRhs;
			z *= rRhs;
		}
	}

	double length() {
		return sqrt(x * x + y * y + z * z);
	}

	// normalization
	void normalize() {
		double len = length();
		if (len) {
			double rLen = 1.0 / len;
			x *= rLen;
			y *= rLen;
			z *= rLen;
		}
	}

	// 常用颜色
	static Vector3 red() { return Vector3(1.0, 0.0, 0.0); };
	static Vector3 green() { return Vector3(0.0, 1.0, 0.0); };
	static Vector3 blue() { return Vector3(0.0, 0.0, 1.0); };
	static Vector3 black() { return Vector3(0.0, 0.0, 0.0); };
	static Vector3 white() { return Vector3(1.0, 1.0, 1.0); };
};

inline ostream& operator << (ostream& os, const Vector3& vec) {
	cout << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
	return os;
}

inline Vector3 operator * (const double k, const Vector3& vec) {
	return Vector3(k * vec.x, k * vec.y, k * vec.z);
}

inline Vector3 operator * (const Vector3& vec, const double k) {
	return Vector3(k * vec.x, k * vec.y, k * vec.z);
}

inline Vector3 operator / (const Vector3& vec, const double k) {
	if (k)
		return Vector3(vec.x / k, vec.y / k, vec.z / k);
}

// 点乘
inline double dotProduct(const Vector3& lhs, const Vector3& rhs) {
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

// 叉乘
inline Vector3 crossProduct(const Vector3& lhs, const Vector3& rhs) {
	return Vector3(
		lhs.y * rhs.z - lhs.z * rhs.y,
		lhs.z * rhs.x - lhs.x * rhs.z,
		lhs.x * rhs.y - lhs.y * rhs.x);
}


// 四维向量
// 表示齐次坐标下的点或向量
class Vector4 {
public:
	double x;
	double y;
	double z;
	double w;

	Vector4() : x(0.0), y(0.0), z(0.0), w(0.0) {}
	Vector4(const double ix, const double iy, const double iz, const double iw)
		: x(ix), y(iy), z(iz), w(iw) {}

public:
	Vector4 operator - () const {
		return Vector4(-x, -y, -z, -w);
	}

	Vector4 operator + (const Vector4& rhs) const {
		return Vector4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
	}

	Vector4 operator - (const Vector4& rhs) const {
		return Vector4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
	}

	void operator += (const Vector4& rhs) {
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		w += rhs.w;
	}

	void operator -= (const Vector4& rhs) {
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		w -= rhs.w;
	}

	void operator *= (const double rhs) {
		x *= rhs;
		y *= rhs;
		z *= rhs;
		w *= rhs;
	}

	void operator /= (const double rhs) {
		x /= rhs;
		y /= rhs;
		z /= rhs;
		w /= rhs;
	}

	double length() const {
		return sqrt(x * x + y * y + z * z);
	}

	// normalization
	void normalize() {
		double len = length();
		if (len) {
			double rLen = 1.0 / len;
			x *= rLen;
			y *= rLen;
			z *= rLen;
		}
	}

	void divideW() {
		if (w == 0.0 || w == 1.0)
			return;
		double rhw = 1.0 / w;
		x *= rhw;
		y *= rhw;
		z *= rhw;
		w = 1.0;
	}
};

inline ostream& operator << (ostream& os, const Vector4& vec) {
	cout << "(" << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w << ")";
	return os;
}

inline Vector4 operator * (const double k, const Vector4& vec) {
	return Vector4(k * vec.x, k * vec.y, k * vec.z, k * vec.w);
}

inline Vector4 operator * (const Vector4& vec, const double k) {
	return Vector4(k * vec.x, k * vec.y, k * vec.z, k * vec.w);
}

inline Vector4 operator / (const Vector4& vec, const double k) {
	return Vector4(vec.x / k, vec.y / k, vec.z / k, vec.w / k);
}

// 点乘（不带w）
inline double dotProduct(const Vector4& lhs, const Vector4& rhs) {
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

// 叉乘（不带w）
inline Vector4 crossProduct(const Vector4& lhs, const Vector4& rhs) {
	return Vector4(
		lhs.y * rhs.z - lhs.z * rhs.y,
		lhs.z * rhs.x - lhs.x * rhs.z,
		lhs.x * rhs.y - lhs.y * rhs.x,
		0.0);
}

// 4 * 4 的矩阵
class Matrix {
public:
	double mat[4][4];

	// 默认初始化成零矩阵
	Matrix() {
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				mat[i][j] = 0;
	}
	Matrix(const std::initializer_list<double>& il) {
		if (il.size() != 16) {
			throw "Matrix need 16 numbers.";
		}
		auto it = il.begin();
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				mat[i][j] = *it++;
	}

	// 矩阵 * 矩阵
	Matrix operator * (const Matrix& rhs) const {
		Matrix ans;
		for (int k = 0; k < 4; ++k) {
			for (int i = 0; i < 4; ++i) {
				double r = mat[i][k];
				for (int j = 0; j < 4; ++j)
					ans.mat[i][j] += r * rhs.mat[k][j];
			}
		}
		return ans;
	}

	// 矩阵 * 向量
	Vector4 operator * (const Vector4& rhs) const {
		Vector4 ans;
		double temp[4] = { rhs.x, rhs.y, rhs.z, rhs.w };
		for (int i = 0; i < 4; ++i)
			ans.x += mat[0][i] * temp[i];
		for (int i = 0; i < 4; ++i)
			ans.y += mat[1][i] * temp[i];
		for (int i = 0; i < 4; ++i)
			ans.z += mat[2][i] * temp[i];
		for (int i = 0; i < 4; ++i)
			ans.w += mat[3][i] * temp[i];
		return ans;
	}

	// 零矩阵
	void setZero() {
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				mat[i][j] = 0;
	}

	// 单位矩阵
	void setIdentity() {
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				mat[i][j] = i == j ? 1.0 : 0.0;
	}

	// 平移矩阵
	void setTranslate(const double x, const double y, const double z) {
		setIdentity();
		mat[0][3] = x;
		mat[1][3] = y;
		mat[2][3] = z;
	}

	// 放缩矩阵
	void setScale(const double x, const double y, const double z) {
		mat[0][0] = x;
		mat[1][1] = y;
		mat[2][2] = z;
		mat[3][3] = 1.0;
	}

	// 旋转矩阵，绕(X, Y, Z)轴旋转theta弧度
	void setRotate(const double X, const double Y, const double Z, const double theta) {
		double qsin = sin(theta * 0.5);
		double qcos = cos(theta * 0.5);
		Vector3 vec(X, Y, Z);
		vec.normalize();
		double w = qcos;
		double x = vec.x * qsin;
		double y = vec.y * qsin;
		double z = vec.z * qsin;
		mat[0][0] = 1 - 2 * y * y - 2 * z * z;
		mat[1][0] = 2 * x * y - 2 * w * z;
		mat[2][0] = 2 * x * z + 2 * w * y;
		mat[0][1] = 2 * x * y + 2 * w * z;
		mat[1][1] = 1 - 2 * x * x - 2 * z * z;
		mat[2][1] = 2 * y * z - 2 * w * x;
		mat[0][2] = 2 * x * z - 2 * w * y;
		mat[1][2] = 2 * y * z + 2 * w * x;
		mat[2][2] = 1 - 2 * x * x - 2 * y * y;
		mat[0][3] = mat[1][3] = mat[2][3] = 0.0;
		mat[3][0] = mat[3][1] = mat[3][2] = 0.0;
		mat[3][3] = 1.0;
	}

	void setRotate(const Vector3& axis, const double theta) {
		setRotate(axis.x, axis.y, axis.z, theta);
	}

	void setRotate(const Vector4& axis, const double theta) {
		setRotate(axis.x, axis.y, axis.z, theta);
	}
};

inline ostream& operator << (ostream& os, const Matrix& tm) {
	cout << "[" << endl;
	for (int i = 0; i < 4; ++i) {
		cout << " ";
		for (int j = 0; j < 4; ++j) {
			cout << tm.mat[i][j] << ", ";
		}
		cout << endl;
	}
	cout << "]";
	return os;
}

// double * 矩阵
inline Matrix operator * (const double lhs, const Matrix& rhs) {
	Matrix ans;
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			ans.mat[i][j] = lhs * rhs.mat[i][j];
	return ans;
}

// 矩阵 * double
inline Matrix operator * (const Matrix& lhs, const double rhs) {
	Matrix ans;
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			ans.mat[i][j] = rhs * lhs.mat[i][j];
	return ans;
}
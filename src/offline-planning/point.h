/*****************************************************************//**
 * \file   point.h
 * \brief  2D and 3D point
 * 
 * \author Qiuchen Qian
 * \date   April 2024
 *********************************************************************/
#ifndef __POINT_H__
#define __POINT_H__

#include "utils.h"

struct Point2D
{
	Point2D() = default;
	Point2D(double x_, double y_) : x(x_), y(y_){}
	Point2D(const Point2D& p) : x(p.x), y(p.y) {}
	~Point2D() {}

	double x = 0;
	double y = 0;

	double sqSum() const { return pow(x, 2) + pow(y, 2); }
	double norm2() const { return std::hypot(x, y); }
	double innerProd(const Point2D& pt) const { return x * pt.x + y * pt.y; }

	Point2D operator+(const Point2D& p) const { return Point2D(x + p.x, y + p.y); }
	Point2D operator-(const Point2D& p) const { return Point2D(x - p.x, y - p.y); }
	Point2D operator*(const double val) const { return Point2D(x * val, y * val); }
	Point2D operator/(const double val) const { return Point2D(x / val, y / val); }
	Point2D& operator=(const Point2D& p) { x = p.x;	y = p.y; return *this; }
	Point2D& operator+=(const Point2D& p) { x += p.x; y += p.y; return *this; }
	Point2D& operator-=(const Point2D& p) { x -= p.x; y -= p.y; return *this; }
	Point2D& operator*=(const double val) { x *= val; y *= val; return *this; }
	Point2D& operator/=(const double val) { x /= val; y /= val; return *this; }
};


struct Point3D
{
	Point3D() = default;
	Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
	Point3D(const Point3D& p) : x(p.x), y(p.y), z(p.z) {}
	~Point3D() {}

	double x = 0;
	double y = 0;
	double z = 0;

	double distXY(const Point3D& pt) const { return std::hypot(x - pt.x, y - pt.y); }
	double distZ(const Point3D& pt) const { return abs(z - pt.z); }
	double sqSum() const { return pow(x, 2) + pow(y, 2) + pow(z, 2); }
	double norm2() const { return std::hypot(x, y, z); }
	double innerProd(const Point3D& pt) const { return x * pt.x + y * pt.y + z * pt.z; }

	bool operator==(const Point3D& p) const 
	{
		return abs(x - p.x) <= 1e-4 && abs(y - p.y) <= 1e-4 && abs(z - p.z) <= 1e-4;
	}
	Point3D operator+(const Point3D& p) const { return Point3D(x + p.x, y + p.y, z + p.z); }
	Point3D operator-(const Point3D& p) const { return Point3D(x - p.x, y - p.y, z - p.z); }
	Point3D operator*(const double val) const { return Point3D(x * val, y * val, z * val); }
	Point3D operator/(const double val) const { return Point3D(x / val, y / val, z / val); }
	Point3D& operator=(const Point3D& p) { x = p.x;	y = p.y; z = p.z; return *this; }
	Point3D& operator+=(const Point3D& p) { x += p.x; y += p.y; z += p.z; return *this; }
	Point3D& operator-=(const Point3D& p) { x -= p.x; y -= p.y; z -= p.z; return *this; }
	Point3D& operator*=(const double val) { x *= val; y *= val; z *= val; return *this; }
	Point3D& operator/=(const double val) { x /= val; y /= val; z /= val; return *this; }
}; 

#endif // !__POINT_H__


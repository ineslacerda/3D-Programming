#ifndef AABB_H
#define AABB_H

#include "vector.h"
#include "boundingBox.h"

//-------------------------------------------------------------------- - default constructor
AABB::AABB(void) 
{
	min = Vector(-1.0f, -1.0f, -1.0f);
	max = Vector(1.0f, 1.0f, 1.0f);
}

// --------------------------------------------------------------------- constructor
AABB::AABB(const Vector& v0, const Vector& v1)
{
	min = v0; max = v1;
}

// --------------------------------------------------------------------- copy constructor
AABB::AABB(const AABB& bbox) 
{
	min = bbox.min; max = bbox.max;
}

// --------------------------------------------------------------------- assignment operator
AABB AABB::operator= (const AABB& rhs) {
	if (this == &rhs)
		return (*this);
	min = rhs.min;
	max = rhs.max;
	return (*this);
}

// --------------------------------------------------------------------- destructor
AABB::~AABB() {}

// --------------------------------------------------------------------- inside
// used to test if a ray starts inside a grid

bool AABB::isInside(const Vector& p) 
{
	return ((p.x > min.x && p.x < max.x) && (p.y > min.y && p.y < max.y) && (p.z > min.z && p.z < max.z));
}

bool AABB::intercepts(const Ray& ray, float& t0, float& t1, Vector& tmin, Vector& tmax)
{
	float ox = ray.origin.x; float oy = ray.origin.y; float oz = ray.origin.z;
	float dx = ray.direction.x; float dy = ray.direction.y; float dz = ray.direction.z;

	double a = 1.0 / dx;
	if (a >= 0) {
		tmin.x = (min.x - ox) * a;
		tmax.x = (max.x - ox) * a;
	}
	else {
		tmin.x = (max.x - ox) * a;
		tmax.x = (min.x - ox) * a;
	}

	double b = 1.0 / dy;
	if (b >= 0) {
		tmin.y = (min.y - oy) * b;
		tmax.y = (max.y - oy) * b;
	}
	else {
		tmin.y = (max.y - oy) * b;
		tmax.y = (min.y - oy) * b;
	}

	double c = 1.0 / dz;
	if (c >= 0) {
		tmin.z = (min.z - oz) * c;
		tmax.z = (max.z - oz) * c;
	}
	else {
		tmin.z = (max.z - oz) * c;
		tmax.z = (min.z - oz) * c;
	}

	//find largest entering t value

	if (tmin.x > tmin.y)
		t0 = tmin.x;
	else
		t0 = tmin.y;

	if (tmin.z > t0)
		t0 = tmin.z;

	//find smallest exiting t value

	if (tmax.x < tmax.y)
		t1 = tmax.x;
	else
		t1 = tmax.y;

	if (tmax.z < t1)
		t1 = tmax.z;


	if (t0 > t1) return false;
	if (t1 < 0) return false;
	
	if (t0 < t1 && t1 >= 0) return true;

}
#endif
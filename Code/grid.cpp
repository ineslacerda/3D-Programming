#include <iostream>
#include <string>
#include <fstream>
#include <IL/il.h>

#include "grid.h"
#include "scene.h"
#include "maths.h"

Grid::Grid(vector<Object*> sceneObjects)
{
	for (int j = 0; j < sceneObjects.size(); j++)
		addObject(sceneObjects[j]);

	Build();
}

int Grid::getNumObjects()
{
	return objects.size();
}

void Grid::addObject(Object* o)
{
	objects.push_back(o);
}

Object* Grid::getObject(unsigned int index)
{
	if (index >= 0 && index < objects.size())
		return objects[index];
	return NULL;
}

int Grid::getNumCells()
{
	return cells.size();
}

Vector Grid::find_min_bounds()
{
	AABB box;
	Vector p0 = Vector(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());

	int num_objects = getNumObjects();
	
	for (int j = 0; j < num_objects; j++) {
		Object* object = getObject(j);
		box = object->GetBoundingBox();

		if (box.min.x < p0.x)
			p0.x = box.min.x;
		if (box.min.y < p0.y)
			p0.y = box.min.y;
		if (box.min.z < p0.z)
			p0.z = box.min.z;
	}
	p0.x -= EPSILON; p0.y -= EPSILON; p0.z -= EPSILON;

	return (p0);
}

Vector Grid::find_max_bounds()
{
	AABB box;
	Vector p1 = Vector(numeric_limits<float>::min(), numeric_limits<float>::min(), numeric_limits<float>::min());

	for (int j = 0; j < getNumObjects(); j++) {
		Object* object = getObject(j);
		box = object->GetBoundingBox();

		if (box.max.x > p1.x)
			p1.x = box.max.x;
		if (box.max.y > p1.y)
			p1.y = box.max.y;
		if (box.max.z > p1.z)
			p1.z = box.max.z;
	}
	p1.x += EPSILON; p1.y += EPSILON; p1.z += EPSILON;

	return (p1);
}

void Grid::Build()
{	
	bbox.max = find_max_bounds();
	bbox.min = find_min_bounds();
	Vector dim = bbox.max - bbox.min;

	float S = pow(getNumObjects() / (dim.x * dim.y * dim.z), 1.0f/3.0f);

	nx = trunc(m * dim.x * S) + 1;
	ny = trunc(m * dim.y * S) + 1;
	nz = trunc(m * dim.z * S) + 1;

	int totalcells = nx * ny * nz;
	for (int i = 0; i < totalcells; i++) {
		vector<Object*> cell;
		cells.push_back(cell);
	}

	for (int i = 0; i < getNumObjects(); i++) {
		Vector objBBmin = getObject(i)->GetBoundingBox().min;
		Vector objBBmax = getObject(i)->GetBoundingBox().max;

		int ixmin = clamp((int)((objBBmin.x - bbox.min.x) * nx / dim.x), 0, int(nx - 1));
		int iymin = clamp((int)((objBBmin.y - bbox.min.y) * ny / dim.y), 0, int(ny - 1));
		int izmin = clamp((int)((objBBmin.z - bbox.min.z) * nz / dim.z), 0, int(nz - 1));

		int ixmax = clamp((int)((objBBmax.x - bbox.min.x) * nx / dim.x), 0, int(nx - 1));
		int iymax = clamp((int)((objBBmax.y - bbox.min.y) * ny / dim.y), 0, int(ny - 1));
		int izmax = clamp((int)((objBBmax.z - bbox.min.z) * nz / dim.z), 0, int(nz - 1));

		for (int iz = izmin; iz <= izmax; iz++) {
			for (int iy = iymin; iy <= iymax; iy++) {
				for (int ix = ixmin; ix <= ixmax; ix++) {
					int index = ix + nx * iy + nx * ny * iz;
					cells[index].push_back(getObject(i));
				}
			}
		}
	}
}

void Grid::Init_Traverse(float dx, float& index, double& dtx, float& t_next, float& i_step, float& i_stop, float& tmin, float& tmax, int nx) {
	if (dx > 0) {
		t_next = tmin + (index + 1) * dtx;
		i_step = +1;
		i_stop = nx;
	}
	else {
		t_next = tmin + (nx - index) * dtx;
		i_step = -1;
		i_stop = -1;
	}

	if (dx == 0.0) {
		t_next = numeric_limits<float>::max();
		i_step = -1;
		i_stop = -1;
	}
}

Object* Grid::Traverse(Ray& ray, float& tNear)
{
	float ox = ray.origin.x; float oy = ray.origin.y; float oz = ray.origin.z;
	float dx = ray.direction.x; float dy = ray.direction.y; float dz = ray.direction.z;

	Vector tmin;
	Vector tmax;

	float t0 = numeric_limits<float>::min();
	float t1 = numeric_limits<float>::max();

	if (!bbox.intercepts(ray, t0, t1, tmin, tmax)) return nullptr;
	
	Vector index; //starting cell indices

	// checks if ray starts inside the grid
	if (bbox.isInside(ray.origin)) {
		index.x = clamp((int)((ox - bbox.min.x) * nx / (bbox.max.x - bbox.min.x)), 0, int(nx - 1));
		index.y = clamp((int)((oy - bbox.min.y) * ny / (bbox.max.y - bbox.min.y)), 0, int(ny - 1));
		index.z = clamp((int)((oz - bbox.min.z) * nz / (bbox.max.z - bbox.min.z)), 0, int(nz - 1));
	}
	else {
		// initial hit point with grid's bounding box
		Vector p = ray.origin + ray.direction * t0;
		index.x = clamp((int)((p.x - bbox.min.x) * nx / (bbox.max.x - bbox.min.x)), 0, int(nx - 1));
		index.y = clamp((int)((p.y - bbox.min.y) * ny / (bbox.max.y - bbox.min.y)), 0, int(ny - 1));
		index.z = clamp((int)((p.z - bbox.min.z) * nz / (bbox.max.z - bbox.min.z)), 0, int(nz - 1));
	}

	// ray parameter increments per cell in the x, y, and z directions
	double dtx = (tmax.x - tmin.x) / nx;
	double dty = (tmax.y - tmin.y) / ny;
	double dtz = (tmax.z - tmin.z) / nz;

	Vector t_next, i_step, i_stop;

	Init_Traverse(dx, index.x, dtx, t_next.x, i_step.x, i_stop.x, tmin.x, tmax.x, nx);
	Init_Traverse(dy, index.y, dty, t_next.y, i_step.y, i_stop.y, tmin.y, tmax.y, ny);
	Init_Traverse(dz, index.z, dtz, t_next.z, i_step.z, i_stop.z, tmin.z, tmax.z, nz);

	// Traverse the grid
	while (true) {
		int cellIndex = index.x + nx * index.y + nx * ny * index.z;
		vector<Object*> cell = cells[cellIndex];
		Object* hitobject = nullptr;
		float tNearaux = INFINITY;
		float taux;

		// checks intercection with objects
		for (int i = 0; i < cell.size(); i++) {
			if (cell[i]->intercepts(ray, taux) && taux < tNearaux) {
				tNearaux = taux;
				hitobject = cell[i];
				tNear = tNearaux;
			}
		}
		
		if (t_next.x < t_next.y && t_next.x < t_next.z) {
			if (hitobject && tNearaux < t_next.x) return hitobject;
			t_next.x += dtx;
			index.x += i_step.x;

			if (index.x == i_stop.x)
				return nullptr;
		}
		else {
			if (t_next.y < t_next.z) {
				if (hitobject && tNearaux < t_next.y) return hitobject;
				t_next.y += dty;
				index.y += i_step.y;

				if (index.y == i_stop.y)
					return nullptr;
			}
			else {
				if (hitobject && tNearaux < t_next.z) return hitobject;
				t_next.z += dtz;
				index.z += i_step.z;

				if (index.z == i_stop.z)
					return nullptr;
			}
		}
	}
}

bool Grid::TraverseShadow(Ray& ray) {
	float ox = ray.origin.x; float oy = ray.origin.y; float oz = ray.origin.z;
	float dx = ray.direction.x; float dy = ray.direction.y; float dz = ray.direction.z;

	Vector tmin;
	Vector tmax;

	float t0 = numeric_limits<float>::min();
	float t1 = numeric_limits<float>::max();

	if (!bbox.intercepts(ray, t0, t1, tmin, tmax)) return false;

	Vector index; //starting cell indices

	if (bbox.isInside(ray.origin)) {  			// does the ray start inside the grid?
		index.x = clamp((int)((ox - bbox.min.x) * nx / (bbox.max.x - bbox.min.x)), 0, int(nx - 1));
		index.y = clamp((int)((oy - bbox.min.y) * ny / (bbox.max.y - bbox.min.y)), 0, int(ny - 1));
		index.z = clamp((int)((oz - bbox.min.z) * nz / (bbox.max.z - bbox.min.z)), 0, int(nz - 1));
	}
	else {
		Vector p = ray.origin + ray.direction * t0;  // initial hit point with grid's bounding box
		index.x = clamp((int)((p.x - bbox.min.x) * nx / (bbox.max.x - bbox.min.x)), 0, int(nx - 1));
		index.y = clamp((int)((p.y - bbox.min.y) * ny / (bbox.max.y - bbox.min.y)), 0, int(ny - 1));
		index.z = clamp((int)((p.z - bbox.min.z) * nz / (bbox.max.z - bbox.min.z)), 0, int(nz - 1));
	}

	// ray parameter increments per cell in the x, y, and z directions

	double dtx = (tmax.x - tmin.x) / nx;
	double dty = (tmax.y - tmin.y) / ny;
	double dtz = (tmax.z - tmin.z) / nz;

	Vector t_next, i_step, i_stop;

	Init_Traverse(dx, index.x, dtx, t_next.x, i_step.x, i_stop.x, tmin.x, tmax.x, nx);
	Init_Traverse(dy, index.y, dty, t_next.y, i_step.y, i_stop.y, tmin.y, tmax.y, ny);
	Init_Traverse(dz, index.z, dtz, t_next.z, i_step.z, i_stop.z, tmin.z, tmax.z, nz);
	
	while (true) {
		int cellIndex = index.x + nx * index.y + nx * ny * index.z;
		vector<Object*> cell = cells[cellIndex];
		Object* hitobject = nullptr;
		float taux;

		for (int i = 0; i < cell.size(); i++)
			if (cell[i]->intercepts(ray, taux)) return true;

		if (t_next.x < t_next.y && t_next.x < t_next.z) {
			t_next.x += dtx;
			index.x += i_step.x;

			if (index.x == i_stop.x)
				return false;
		}
		else {
			if (t_next.y < t_next.z) {
				t_next.y += dty;
				index.y += i_step.y;

				if (index.y == i_stop.y)
					return false;
			}
			else {
				t_next.z += dtz;
				index.z += i_step.z;

				if (index.z == i_stop.z)
					return false;
			}
		}
	}
}


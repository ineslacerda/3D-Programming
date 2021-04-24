#ifndef GRID_H
#define GRID_H

#include <vector>
#include <cmath>
#include "scene.h"

using namespace std;

class Grid
{
public:
	Grid(vector<Object*>);
	//~Grid(void);

	int getNumObjects();
	void addObject(Object* o);
	Object* getObject(unsigned int index);

	int getNumCells();

	void Build();   // set up grid cells

	Object* Traverse(Ray& ray, float& t); 
	bool TraverseShadow(Ray& ray); //Traverse for shadow ray

private:
	vector<Object *> objects;
	vector<vector<Object*> > cells;

	int nx, ny, nz; // number of cells in the x, y, and z directions
	float m = 2.0f; // factor that allows to vary the number of cells

	Vector find_min_bounds(void);
	Vector find_max_bounds(void);

	//Setup function for Grid traversal
	void Init_Traverse(float dx, float& index, double& dtx, float& t_next, float& i_step, float& i_stop, float& tmin, float& tmax, int nx);

	AABB bbox;
};
#endif

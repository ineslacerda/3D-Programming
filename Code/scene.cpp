#include <iostream>
#include <string>
#include <fstream>
#include <IL/il.h>

#include "maths.h"
#include "scene.h"

Vector cross_product(Vector vector_a, Vector vector_b) {
	return Vector(
		vector_a.y * vector_b.z - vector_a.z * vector_b.y,
		vector_a.z * vector_b.x - vector_a.x * vector_b.z,
		vector_a.x * vector_b.y - vector_a.y * vector_b.x
	);
}

Triangle::Triangle(Vector& P0, Vector& P1, Vector& P2)
{
	points[0] = P0; points[1] = P1; points[2] = P2;

	/* Calculate the normal */

	Vector edge1 = points[1] - points[0];
	Vector edge2 = points[2] - points[0];
	normal = cross_product(edge1, edge2).normalize();

	//Calculate the Min and Max for bounding box
	Vector min = Vector(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
	Vector max = Vector(numeric_limits<float>::min(), numeric_limits<float>::min(), numeric_limits<float>::min());

	for (int i = 0; i < 3; i++) {
		if (points[i].x < min.x) min.x = points[i].x;
		if (points[i].x > max.x) max.x = points[i].x;
		if (points[i].y < min.y) min.y = points[i].y;
		if (points[i].y > max.y) max.y = points[i].y;
		if (points[i].z < min.z) min.z = points[i].z;
		if (points[i].z > max.z) max.z = points[i].z;
	}

	Min = min;
	Max = max;

	/*Min = Vector(+FLT_MAX, +FLT_MAX, +FLT_MAX);
	Max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);*/


	// enlarge the bounding box a bit just in case...
	Min -= EPSILON;
	Max += EPSILON;
}

AABB Triangle::GetBoundingBox() {
	return(AABB(Min, Max));
}

Vector Triangle::getNormal(Vector point)
{
	return normal;
}

//
// Ray/Triangle intersection test using Tomas Moller-Ben Trumbore algorithm.
//

bool Triangle::intercepts(Ray& r, float& t) {

	float a = points[0].x - points[1].x, b = points[0].x - points[2].x, c = r.direction.x, d = points[0].x - r.origin.x;
	float e = points[0].y - points[1].y, f = points[0].y - points[2].y, g = r.direction.y, h = points[0].y - r.origin.y;
	float i = points[0].z - points[1].z, j = points[0].z - points[2].z, k = r.direction.z, l = points[0].z - r.origin.z;

	float m = f * k - g * j, n = h * k - g * l, p = f * l - h * j;
	float q = g * i - e * k, s = e * j - f * i;

	float inv_denom = 1.0 / (a * m + b * q + c * s);

	float e1 = d * m - b * n - c * p;
	float beta = e1 * inv_denom;

	if (beta < 0.0)
		return false;

	float ray = e * l - h * i;
	float e2 = a * n + d * q + c * ray;
	float gamma = e2 * inv_denom;

	if (gamma < 0.0)
		return false;

	if (beta + gamma > 1.0)
		return false;

	float e3 = a * p - b * ray + d * s;
	t = e3 * inv_denom;

	if (t < 0.0001f)
		return false;

	//sr.local_hit_point = ray.o + t * ray.d;

	return true;

}


Plane::Plane(Vector& a_PN, float a_D)
	: PN(a_PN), D(a_D)
{}

Plane::Plane(Vector& P0, Vector& P1, Vector& P2)
{
	float l;

	//Calculate the normal plane: counter-clockwise vectorial product.
	Vector edge1 = P1 - P0;
	Vector edge2 = P2 - P0;
	PN = cross_product(edge1, edge2).normalize();

	if ((l = PN.length()) == 0.0)
	{
		cerr << "DEGENERATED PLANE!\n";
	}
	else
	{
		PN.normalize();
		//Calculate D
		D = P0 * PN;
	}
}

//
// Ray/Plane intersection test.
//

bool Plane::intercepts(Ray& r, float& t)
{
	float aux = PN * r.direction; //PN is the normal

	//There is no intersection
	if (abs(aux) < 0.0001f) {
		return false;
	}

	t = -((r.origin * PN) - D) / aux; //D is the dot_product between P0 and PN

	if (t > 0.0f) {
		return true;
	}
	return false;
}

Vector Plane::getNormal(Vector point)
{
	return PN;
}

bool Sphere::intercepts(Ray& r, float& t) {
	Vector temp = center - r.origin;
	float b = r.direction * temp;
	float c = temp * temp - SqRadius;

	if (b <= 0.0f) return false;

	float disc = b * b - c;

	if (disc <= 0.0f) return false;

	if (c > 0.0f) {
		//smaller root
		t = b - sqrt(disc);
	}
	else
		//positive root
		t = b + sqrt(disc);

	if (t > 0.0f) {
		return true;
	}

	return false;
}


Vector Sphere::getNormal(Vector point)
{
	Vector normal = point - center;
	return (normal.normalize());
}

AABB Sphere::GetBoundingBox() {
	Vector a_min, a_max;
	a_min.x = center.x - radius;
	a_min.y = center.y - radius;
	a_min.z = center.z - radius;
	a_max.x = center.x + radius;
	a_max.y = center.y + radius;
	a_max.z = center.z + radius;
	return(AABB(a_min, a_max));
}

aaBox::aaBox(Vector& minPoint, Vector& maxPoint) //Axis aligned Box: another geometric object
{
	this->min = minPoint;
	this->max = maxPoint;
}

AABB aaBox::GetBoundingBox() {
	return(AABB(min, max));
}

bool aaBox::intercepts(Ray& ray, float& t)
{
	float ox = ray.origin.x; float oy = ray.origin.y; float oz = ray.origin.z;
	float dx = ray.direction.x; float dy = ray.direction.y; float dz = ray.direction.z;

	float tx_min, ty_min, tz_min;
	float tx_max, ty_max, tz_max;

	double a = 1.0 / dx;
	if (a >= 0) {
		tx_min = (this->min.x - ox) * a;
		tx_max = (this->max.x - ox) * a;
	}
	else {
		tx_min = (this->max.x - ox) * a;
		tx_max = (this->min.x - ox) * a;
	}

	float b = 1.0 / dy;
	if (b >= 0) {
		ty_min = (this->min.y - oy) * b;
		ty_max = (this->max.y - oy) * b;
	}
	else {
		ty_min = (this->max.y - oy) * b;
		ty_max = (this->min.y - oy) * b;
	}

	float c = 1.0 / dz;
	if (c >= 0) {
		tz_min = (this->min.z - oz) * c;
		tz_max = (this->max.z - oz) * c;
	}
	else {
		tz_min = (this->max.z - oz) * c;
		tz_max = (this->min.z - oz) * c;
	}

	float t0, t1;
	Vector face_in, face_out;

	//find largest entering t value

	if (tx_min > ty_min) {
		t0 = tx_min;
		face_in = (a >= 0.0) ? Vector(-1, 0, 0) : Vector(1, 0, 0);
	}
	else {
		t0 = ty_min;
		face_in = (b >= 0.0) ? Vector(0, -1, 0) : Vector(0, 1, 0);
	}

	if (tz_min > t0) {
		t0 = tz_min;
		face_in = (c >= 0.0) ? Vector(0, 0, -1) : Vector(0, 0, 1);
	}

	//find smallest exiting t value

	if (tx_max < ty_max) {
		t1 = tx_max;
		face_out = (a >= 0.0) ? Vector(1, 0, 0) : Vector(-1, 0, 0);
	}
	else {
		t1 = ty_max;
		face_out = (b >= 0.0) ? Vector(0, 1, 0) : Vector(0, -1, 0);
	}

	if (tz_max < t1) {
		t1 = tz_max;
		face_out = (c >= 0.0) ? Vector(0, 0, 1) : Vector(0, 0, -1);
	}
	if (t0 < t1 && t1 >= EPSILON) {
		if (t0 > 0) {
			t = t0;
			Normal = face_in;
		}
		else {
			t = t1;
			Normal = face_out;
		}
		return true;
	}

	return false;
}

Vector aaBox::getNormal(Vector point)
{
	return Normal;
}

Scene::Scene()
{}

Scene::~Scene()
{
	/*for ( int i = 0; i < objects.size(); i++ )
	{
		delete objects[i];
	}
	objects.erase();
	*/
}

vector<Object*>  Scene::getObjects()
{
	return objects;
}


int Scene::getNumObjects()
{
	return objects.size();
}


void Scene::addObject(Object* o)
{
	objects.push_back(o);
}


Object* Scene::getObject(unsigned int index)
{
	if (index >= 0 && index < objects.size())
		return objects[index];
	return NULL;
}


int Scene::getNumLights()
{
	return lights.size();
}


void Scene::addLight(Light* l)
{
	lights.push_back(l);
}


Light* Scene::getLight(unsigned int index)
{
	if (index >= 0 && index < lights.size())
		return lights[index];
	return NULL;
}

void Scene::LoadSkybox(const char* sky_dir)
{
	char* filenames[6];
	char buffer[100];
	//const char* maps[] = { "/background1.jpg", "/background2.jpg", "/background3.jpg", "/background4.jpg", "/background5.jpg", "/background6.jpg" };
	const char* maps[] = { "/right.jpg", "/left.jpg", "/top.jpg", "/bottom.jpg", "/front.jpg", "/back.jpg" };

	for (int i = 0; i < 6; i++) {
		strcpy_s(buffer, sizeof(buffer), sky_dir);
		strcat_s(buffer, sizeof(buffer), maps[i]);
		filenames[i] = (char*)malloc(sizeof(buffer));
		strcpy_s(filenames[i], sizeof(buffer), buffer);
	}

	ILuint ImageName;

	ilEnable(IL_ORIGIN_SET);
	ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

	for (int i = 0; i < 6; i++) {
		ilGenImages(1, &ImageName);
		ilBindImage(ImageName);

		if (ilLoadImage(filenames[i]))  //Image loaded with lower left origin
			printf("Skybox face %d: Image sucessfully loaded.\n", i);
		else
			exit(0);

		ILint bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

		ILenum format = IL_RGB;
		printf("bpp=%d\n", bpp);
		if (bpp == 24)
			format = IL_RGB;
		else if (bpp == 32)
			format = IL_RGBA;

		ilConvertImage(format, IL_UNSIGNED_BYTE);

		int size = ilGetInteger(IL_IMAGE_SIZE_OF_DATA);
		skybox_img[i].img = (ILubyte*)malloc(size);
		ILubyte* bytes = ilGetData();
		memcpy(skybox_img[i].img, bytes, size);
		skybox_img[i].resX = ilGetInteger(IL_IMAGE_WIDTH);
		skybox_img[i].resY = ilGetInteger(IL_IMAGE_HEIGHT);
		format == IL_RGB ? skybox_img[i].BPP = 3 : skybox_img[i].BPP = 4;
		ilDeleteImages(1, &ImageName);
	}
	ilDisable(IL_ORIGIN_SET);
}

Color Scene::GetSkyboxColor(Ray& r) {
	float t_intersec;
	Vector cubemap_coords; //To index the skybox

	float ma;
	CubeMap img_side;
	float sc, tc, s, t;
	unsigned int xp, yp, width, height, bytesperpixel;

	//skybox indexed by the ray direction
	cubemap_coords = r.direction;


	if (fabs(cubemap_coords.x) > fabs(cubemap_coords.y)) {
		ma = fabs(cubemap_coords.x);
		cubemap_coords.x >= 0 ? img_side = LEFT : img_side = RIGHT;    //left cubemap at X = +1 and right at X = -1
	}
	else {
		ma = fabs(cubemap_coords.y);
		cubemap_coords.y >= 0 ? img_side = TOP : img_side = BOTTOM; //top cubemap at Y = +1 and bottom at Y = -1
	}

	if (fabs(cubemap_coords.z) > ma) {
		ma = fabs(cubemap_coords.z);
		cubemap_coords.z >= 0 ? img_side = FRONT : img_side = BACK;   //front cubemap at Z = +1 and back at Z = -1
	}

	switch (img_side) {

	case 0:  //right
		sc = -cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 1:  //left
		sc = cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 2:  //top
		sc = -cubemap_coords.x;
		tc = -cubemap_coords.z;
		break;

	case 3: //bottom
		sc = -cubemap_coords.x;
		tc = cubemap_coords.z;
		break;

	case 4:  //front
		sc = -cubemap_coords.x;
		tc = cubemap_coords.y;
		break;

	case 5: //back
		sc = cubemap_coords.x;
		tc = cubemap_coords.y;
		break;
	}

	double invMa = 1 / ma;
	s = (sc * invMa + 1) / 2;
	t = (tc * invMa + 1) / 2;

	width = skybox_img[img_side].resX;
	height = skybox_img[img_side].resY;
	bytesperpixel = skybox_img[img_side].BPP;

	xp = int((width - 1) * s);
	xp < 0 ? 0 : (xp > (width - 1) ? width - 1 : xp);
	yp = int((height - 1) * t);
	yp < 0 ? 0 : (yp > (height - 1) ? height - 1 : yp);

	float red = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel]);
	float green = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel + 1]);
	float blue = u8tofloat(skybox_img[img_side].img[(yp * width + xp) * bytesperpixel + 2]);

	return(Color(red, green, blue));
}




////////////////////////////////////////////////////////////////////////////////
// P3F file parsing methods.
//
void next_token(ifstream& file, char* token, const char* name)
{
	file >> token;
	if (strcmp(token, name))
		cerr << "'" << name << "' expected.\n";
}

bool Scene::load_p3f(const char* name)
{
	const	int	lineSize = 1024;
	string	cmd;
	char		token[256];
	ifstream	file(name, ios::in);
	Material* material;

	material = NULL;

	if (file >> cmd)
	{
		while (true)
		{

			if (cmd == "f")   //Material
			{
				double Kd, Ks, Shine, T, ior;
				Color cd, cs;

				file >> cd >> Kd >> cs >> Ks >> Shine >> T >> ior;

				material = new Material(cd, Kd, cs, Ks, Shine, T, ior);
			}

			else if (cmd == "s")    //Sphere
			{
				Vector center;
				float radius;
				Sphere* sphere;

				file >> center >> radius;
				sphere = new Sphere(center, radius);
				if (material) sphere->SetMaterial(material);
				this->addObject((Object*)sphere);
			}

			else if (cmd == "box")    //axis aligned box
			{
				Vector minpoint, maxpoint;
				aaBox* box;

				file >> minpoint >> maxpoint;
				box = new aaBox(minpoint, maxpoint);
				if (material) box->SetMaterial(material);
				this->addObject((Object*)box);
			}
			else if (cmd == "p")  // Polygon: just accepts triangles for now
			{
				Vector P0, P1, P2;
				Triangle* triangle;
				unsigned total_vertices;

				file >> total_vertices;
				if (total_vertices == 3)
				{
					file >> P0 >> P1 >> P2;
					triangle = new Triangle(P0, P1, P2);
					if (material) triangle->SetMaterial(material);
					this->addObject((Object*)triangle);
				}
				else
				{
					cerr << "Unsupported number of vertices.\n";
					break;
				}
			}

			else if (cmd == "pl")  // General Plane
			{
				Vector P0, P1, P2;
				Plane* plane;

				file >> P0 >> P1 >> P2;
				plane = new Plane(P0, P1, P2);
				if (material) plane->SetMaterial(material);
				this->addObject((Object*)plane);
			}

			else if (cmd == "l")  // Need to check light color since by default is white
			{
				Vector pos;
				Color color;

				file >> pos >> color;

				this->addLight(new Light(pos, color));

			}
			else if (cmd == "v")
			{
				Vector up, from, at;
				float fov, hither;
				int xres, yres;
				Camera* camera;
				float focal_ratio; //ratio beteween the focal distance and the viewplane distance
				float aperture_ratio; // number of times to be multiplied by the size of a pixel

				next_token(file, token, "from");
				file >> from;

				next_token(file, token, "at");
				file >> at;

				next_token(file, token, "up");
				file >> up;

				next_token(file, token, "angle");
				file >> fov;

				next_token(file, token, "hither");
				file >> hither;

				next_token(file, token, "resolution");
				file >> xres >> yres;

				next_token(file, token, "aperture");
				file >> aperture_ratio;

				next_token(file, token, "focal");
				file >> focal_ratio;
				// Create Camera
				camera = new Camera(from, at, up, fov, hither, 100.0 * hither, xres, yres, aperture_ratio, focal_ratio);
				this->SetCamera(camera);
			}

			else if (cmd == "bclr")   //Background color
			{
				Color bgcolor;
				file >> bgcolor;
				this->SetBackgroundColor(bgcolor);
			}

			else if (cmd == "env")
			{
				file >> token;

				this->LoadSkybox(token);
				this->SetSkyBoxFlg(true);
			}
			else if (cmd[0] == '#')
			{
				file.ignore(lineSize, '\n');
			}
			else
			{
				cerr << "unknown command '" << cmd << "'.\n";
				break;
			}
			if (!(file >> cmd))
				break;
		}
	}

	file.close();

	return true;
};

///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2019 by João Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <conio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "grid.h"
#include "maths.h"
#include "sampler.h"

#define CAPTION "Whitted Ray-Tracer"

#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

#define MAX_DEPTH 4

//antialiasing
bool ANTIALIASING = false;
#define SPP_N 4

//Depth of Field
bool DOF = false;

//Soft shadows
bool SOFTSHADOWS = false;
#define SL_N 8 //N source points for Area Light

//Skybox
bool SKYBOX = false;

//Grid
bool HASGRID = false;

//Enable OpenGL drawing.  
bool drawModeEnabled = true;

//Draw Mode: 0 - point by point; 1 - line by line; 2 - full frame at once
int draw_mode = 1;

// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float* colors;
float* vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t* img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;
Grid* grid = NULL;
int RES_X, RES_Y;

int WindowHandle = 0;

Vector refract(Vector rayDirection, Vector normal, float eta_in, float eta_out) {
	float eta = eta_in / eta_out;

	Vector t, vt, direction;
	float sin, cos;
	Vector negDirection = rayDirection * (-1);
	vt = normal * (negDirection * normal) - negDirection;
	sin = eta * vt.length(); // sin = ||vt||

	cos = sqrt(max(0.0f, 1.0f - (double)sin * sin));
	t = vt / vt.length(); // t = (1/||vt||) * vt
	direction = t * sin + (normal * (-1)) * cos;

	return direction;
}

float fresnel(Vector rayDirection, Vector normal, float eta_in, float eta_out) {
	float kr;
	float cosi = clamp(-1, 1, rayDirection * normal);
	float etai = 1, etat = eta_out;
	if (cosi > 0) { std::swap(etai, etat); }
	// Compute sin using Snell's law
	float sin = etai / etat * sqrtf(max(0.f, 1 - (double)cosi * cosi));
	// Total internal reflection
	if (sin >= 1) {
		kr = 1;
	}
	else {
		float cost = sqrtf(max(0.0f, 1.0f - (double)sin * sin));
		cosi = fabsf(cosi);
		float Rs = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
		float Rp = ((etai * cost) - (etat * cosi)) / ((etai * cost) + (etat * cosi));
		kr = (Rs * Rs + Rp * Rp) / 2;
	}
	return kr;
}

Vector randomPointOnSphere(Sphere S) {
	double randX = pow(-1.0, (rand() % 2 + 1)) * (rand() % 100);
	double randY = pow(-1.0, (rand() % 2 + 1)) * (rand() % 100);
	double randZ = pow(-1.0, (rand() % 2 + 1)) * (rand() % 100);
	Vector randXYZ = Vector(randX, randY, randZ);
	Vector randVector = (randXYZ - S.center).normalize();
	Vector temp = randVector * S.radius;
	Vector randPoint = S.center + temp;
	return randPoint;
}

Vector pointOnSphere(Sphere S, int k) {
	double randX = k;
	double randY = k;
	double randZ = k;
	Vector randXYZ = Vector(randX, randY, randZ);
	Vector randVector = (randXYZ - S.center).normalize();
	Vector temp = randVector * S.radius;
	Vector randPoint = S.center + temp;
	return randPoint;
}

bool shadowRayTracing(Ray shadowRay) {
	if (HASGRID) {
		if (grid->TraverseShadow(shadowRay))
			return true;
		return false;
	}
	else {
		int n = 0;
		float t;
		while (n < scene->getNumObjects()) {
			if (scene->getObject(n)->intercepts(shadowRay, t)) {
				return true;
			}
			n++;
		}
		return false;
	}
}

Color calculateBlinnPhong(Vector lightPosition, Color lightColor, Vector  pointOnLight, Vector offset, Vector intersectionPoint, Vector normal, Vector rayDirection, Material* hitObjectMaterial) {
	Color color;
	Vector lightDirection = (lightPosition + pointOnLight - intersectionPoint).normalize();
	Ray shadowRay = Ray(intersectionPoint + offset, lightDirection);
	bool isShadow = shadowRayTracing(shadowRay);
	float diffuse = lightDirection * normal;
	if (diffuse > 0 && !isShadow) {
		color += lightColor * diffuse * hitObjectMaterial->GetDiffuse() * hitObjectMaterial->GetDiffColor();

		if (hitObjectMaterial->GetSpecular() > 0) {
			float Hn = ((lightDirection - rayDirection).normalize()) * normal;
			float specular = pow(max(0, Hn), hitObjectMaterial->GetShine());
			color += lightColor * hitObjectMaterial->GetSpecular() * hitObjectMaterial->GetSpecColor() * specular;
		}
	}
	return color;
}

Color rayTracing(Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{
	int n = 0;

	float tNear = INFINITY;
	float t;

	Object* hitObject = nullptr;

	Color color;

	if (HASGRID) {
		hitObject = grid->Traverse(ray, tNear);
	}
	else {
		while (n < scene->getNumObjects()) {
			if (scene->getObject(n)->intercepts(ray, t)) {
				if (t < tNear) {
					hitObject = scene->getObject(n);
					tNear = t;
				}
			}
			n++;
		}
	}

	if (hitObject == nullptr) {
		if (SKYBOX) {
			scene->SetSkyBoxFlg(SKYBOX);
			return scene->GetSkyboxColor(ray);
		}
		else
			return scene->GetBackgroundColor();
	}
	else {
		Material* hitObjectMaterial = hitObject->GetMaterial();

		Vector intersectionPoint = ray.direction * tNear + ray.origin;
		Vector normal = (hitObject->getNormal(intersectionPoint)).normalize();

		n = 0;

		Vector offset = normal * 0.001f;
		bool inside = ray.direction * normal > 0;

		while (n < scene->getNumLights()) {
			Light* light = scene->getLight(n);
			Vector pointOnLight = Vector(0, 0, 0);
			if (ANTIALIASING) { //random method
				if (SOFTSHADOWS) // gets random point within a sphere
					pointOnLight = randomPointOnSphere(Sphere(light->position, 0.5));

				color += calculateBlinnPhong(light->position, light->color, pointOnLight, offset, intersectionPoint, normal, ray.direction, hitObjectMaterial);
			}
			else {
				if (SOFTSHADOWS)
					// use area light with a set of SL_N light source points
					for (int point = 0; point < SL_N; point++) {
						// creates an area light (sphere)
						pointOnLight = pointOnSphere(Sphere(light->position, 1), point);
						color += calculateBlinnPhong(light->position, light->color, pointOnLight, offset, intersectionPoint, normal, ray.direction, hitObjectMaterial) * (1.0f / SL_N);
					}
				else
					color += calculateBlinnPhong(light->position, light->color, pointOnLight, offset, intersectionPoint, normal, ray.direction, hitObjectMaterial);
			}
			n++;
		}

		if (depth >= MAX_DEPTH) return color;

		//Calculates mirror reflection attenuation using fresnel equations
		float kr = fresnel(ray.direction, normal, ior_1, hitObjectMaterial->GetRefrIndex());

		//Object is reflective
		if (hitObjectMaterial->GetReflection() > 0) {
			Vector reflectedRayDirection = ray.direction - normal * (normal * ray.direction) * 2;
			Ray reflectedRay = Ray(intersectionPoint + offset, reflectedRayDirection);
			Color reflectedColor = rayTracing(reflectedRay, depth + 1, ior_1);
			//Object is reflective and refracted -> use reflection attenuation (fresnel)
			if (hitObjectMaterial->GetTransmittance() > 0) color += reflectedColor * kr;
			else color += reflectedColor * hitObjectMaterial->GetSpecular() * hitObjectMaterial->GetSpecColor();
		}

		// Object is refracted
		if (hitObjectMaterial->GetTransmittance() > 0) {
			float eta_in = ior_1;
			float eta_out = hitObjectMaterial->GetRefrIndex();

			// Check if ray is inside object
			if (inside) {
				normal = normal * (-1);
				eta_out = 1.0;
			}

			Vector direction = refract(ray.direction, normal, eta_in, eta_out);
			Vector refractedRayOrigin;

			if (inside) refractedRayOrigin = intersectionPoint + offset;
			else refractedRayOrigin = intersectionPoint - offset;

			Ray refractedRay = Ray(refractedRayOrigin, direction);
			Color refractedColor = rayTracing(refractedRay, depth + 1, eta_out);
			color += refractedColor * (1 - kr) * hitObjectMaterial->GetTransmittance();
		}

	}

	return color;
}

/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte* errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if (isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");

	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs


void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* Só se faz a alocação dos arrays glBufferData (NULL), e o envio dos pontos para a placa gráfica
	é feito na drawPoints com GlBufferSubData em tempo de execução pois os arrays são GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);

	// unbind the VAO
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
	//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);

	if (draw_mode == 0) glDrawArrays(GL_POINTS, 0, 1);
	else if (draw_mode == 1) glDrawArrays(GL_POINTS, 0, RES_X);
	else glDrawArrays(GL_POINTS, 0, RES_X * RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char* filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	cout << "\nANTIALIASING: " << ANTIALIASING << " DOF: " << DOF << " SOFTSHADOWS: " << SOFTSHADOWS << "\n";
	cout << "\nPress 'a' to switch antialiasing on/off.\nPress 'd' to switch depth of field on/off.\nPress 's' to switch soft shadows on/off.\n" << std::endl;

	int index_pos = 0;
	int index_col = 0;
	unsigned int counter = 0;

	set_rand_seed(time(NULL) * time(NULL));

	grid = new Grid(scene->getObjects());

	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color;

			Vector pixel;  //viewport coordinates
			pixel.x = x + 0.5f;
			pixel.y = y + 0.5f;


			if (!ANTIALIASING) {
				Ray ray = scene->GetCamera()->PrimaryRay(pixel);
				color += rayTracing(ray, 1, 1.0).clamp();
			}
			else { // Has anti-aliasing
				if (DOF) {
					Vector ls = sample_unit_disk() * scene->GetCamera()->GetAperture();
					for (int i = 0; i < SPP_N; i++) {
						for (int j = 0; j < SPP_N; j++) {
							Vector pixel_aux;
							pixel_aux.x = x + (rand_float() + i) / (float)SPP_N;
							pixel_aux.y = y + (rand_float() + j) / (float)SPP_N;
							Ray dofRay = scene->GetCamera()->PrimaryRay(ls, pixel_aux);
							color += rayTracing(dofRay, 1, 1.0).clamp();
						}
					}
				}
				else {
					for (int i = 0; i < SPP_N; i++) {
						for (int j = 0; j < SPP_N; j++) {
							Vector pixel_aux;
							pixel_aux.x = x + (rand_float() + i) / (float)SPP_N;
							pixel_aux.y = y + (rand_float() + j) / (float)SPP_N;
							Ray ray = scene->GetCamera()->PrimaryRay(pixel_aux);
							color += rayTracing(ray, 1, 1.0).clamp();
						}
					}
				}
				color.r(color.r() / (float)pow(SPP_N, 2));
				color.g(color.g() / (float)pow(SPP_N, 2));
				color.b(color.b() / (float)pow(SPP_N, 2));
			}

			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				colors[index_col++] = (float)color.r();

				colors[index_col++] = (float)color.g();

				colors[index_col++] = (float)color.b();


				if (draw_mode == 0) {  // drawing point by point
					drawPoints();
					index_pos = 0;
					index_col = 0;
				}
			}
		}
		if (draw_mode == 1 && drawModeEnabled) {  // drawing line by line
			drawPoints();
			index_pos = 0;
			index_col = 0;
		}
	}
	if (draw_mode == 2 && drawModeEnabled)        //full frame at once
		drawPoints();

	printf("Drawing finished!\n");

	if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
		printf("Error saving Image file\n");
		exit(0);
	}
	printf("Image file created\n");
	glFlush();
}

// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	//destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top,
	float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

	case 27:
		glutLeaveMainLoop();
		break;

	case 97: //a - switch antialiasing on/off
		ANTIALIASING = !ANTIALIASING;
		break;

	case 100: //d - switch depth of field on/off
		DOF = !DOF;
		break;

	case 115: //s - switch soft shadows on/off
		SOFTSHADOWS = !SOFTSHADOWS;
		break;

	}
	renderScene();
}

/////////////////////////////////////////////////////////////////////// SETUP

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
}

void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit();
	if (result != GLEW_OK) {
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	}
	GLenum err_code = glGetError();
	printf("Vendor: %s\n", glGetString(GL_VENDOR));
	printf("Renderer: %s\n", glGetString(GL_RENDERER));
	printf("Version: %s\n", glGetString(GL_VERSION));
	printf("GLSL: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);

	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	glutInitWindowPosition(640, 100);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if (WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}


void init(int argc, char* argv[])
{
	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();

}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	while (true) {
		cout << "Input the Scene Name: ";
		cin >> input_user;
		strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
		strcat_s(scene_name, sizeof(scene_name), input_user);

		ifstream file(scene_name, ios::in);
		if (file.fail()) {
			printf("\nError opening P3F file.\n");
		}
		else
			break;
	}

	scene = new Scene();
	scene->load_p3f(scene_name);
	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X * RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);
}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int ch;
	if (!drawModeEnabled) {

		do {
			init_scene();
			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);

			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while ((toupper(ch) == 'Y'));
	}

	else {   //Use OpenGL to draw image in the screen
		init_scene();
		if (draw_mode == 0) { // draw image point by point
			size_vertices = 2 * sizeof(float);
			size_colors = 3 * sizeof(float);
			printf("DRAWING MODE: POINT BY POINT\n\n");
		}
		else if (draw_mode == 1) { // draw image line by line
			size_vertices = 2 * RES_X * sizeof(float);
			size_colors = 3 * RES_X * sizeof(float);
			printf("DRAWING MODE: LINE BY LINE\n\n");
		}
		else if (draw_mode == 2) { // draw full frame at once
			size_vertices = 2 * RES_X * RES_Y * sizeof(float);
			size_colors = 3 * RES_X * RES_Y * sizeof(float);
			printf("DRAWING MODE: FULL IMAGE\n\n");
		}
		else {
			printf("Draw mode not valid \n");
			exit(0);
		}
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);

		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);

		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////
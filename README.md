####################################
	     READ ME
####################################

Group 1

Inês Lacerda [Nº 86436]
Jin Xin [Nº 86438]
Lucas Soares [Nº 86463]

------------------------------------

------------------------------------
Implementations:
------------------------------------

1) T.Whitted Ray-Tracer
   - Local color component (Blinn-Phong model illumination)
   - Multiple source lights
   - Hard Shadows
   - Global color component by implement the mirror reflection and refraction
   - Ray intersections with spheres, infinite planes and triangles

2) Stochastic sampling techniques
   - Anti-aliasing with the jittered method
   - Soft shadows using an area of light with a set of N light source points         
	   without antialiasing) and the random method (with antialiasing)
   - Depth of field effect where the lens is simulated by a random distribution of         
	   N samples on unit squares or unit disks

3) Acceleration structure
   - Uniform grid - using the Amanatides and Woo (1987) algorithm

4) Extra
   - Box-Ray intersection
   - Calculating mirror reflection attenuation and refraction attenuation 		
	   using Fresnel Equations
   - New Scenes, that can be found in the P3D_Scenes folder with names:
      - new_scene1.p3f
      - new_scene2.p3f
      - new_scene3.p3f

------------------------------------
Compilation:
------------------------------------

1) Use Visual Studio
2) Include il, freeglut and glew dependencies
3) Run with release mode
4) Type a specific scene from the P3D_Scenes folder

----------------------------------------
Change parameters with drawModeEnabled:
----------------------------------------

1) Change drawModeEnabled to true
2) Run program
3) Type specific scene
3) After image is drawn, press image with mouse and type:
	a --> to switch antialiasing on/off
	d --> to switch depth of field on/off
	s --> to switch soft shadows on/off

-------------------------------------
Check execution times:
-------------------------------------

1) Change drawModeEnabled to false
2) Run program
3) Type specific scene
4) Output can be checked in RT_Output.png

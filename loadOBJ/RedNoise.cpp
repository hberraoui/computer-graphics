#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>
#include <fstream>
#include <string.h>

using namespace std;
using namespace glm;

#define WIDTH 320
#define HEIGHT 240

string cornellBoxMtlPath = "../cornell-box/cornell-box.mtl";
string cornellBoxObjPath = "../cornell-box/cornell-box.obj";

void draw();
void update();
void handleEvent(SDL_Event event);
bool loadMtlFile(string filepath);
bool loadObjFile(string filepath);
void centerCameraPosition();
void pointWorldToCameraSpace();

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
vector<Colour> palette;
vector<vec3> vertices;
vector<ModelTriangle> triangles;

float cameraStepBack = -10;
float focalLength;
vec3 pointCameraSpace;
vec3 pointCanvasSpace;
int scale = 20;
vec3 pointWorldSpace;

float centerOfWorld;

int vcounter = 0;
	
vec3 cameraPosition;


// For the sake of Windows
int WinMain(int argc, char* argv[])
{
  main(argc, argv);
  return 0;
}

int main(int argc, char* argv[])
{
	
	loadMtlFile(cornellBoxMtlPath);
	loadObjFile(cornellBoxObjPath);
	// we have world triangles
	
	centerCameraPosition();
	focalLength = abs((cameraPosition.z - centerOfWorld)/2);
	
	cout << "f: " << focalLength << endl;
	
	pointWorldToCameraSpace();
		
		
	
/* 	for (Colour c : palette){
		cout << c << endl; 
	} */
	
	SDL_Event event;
	while(true)
	{
	// We MUST poll for events - otherwise the window will freeze !
	if(window.pollForInputEvents(&event)) handleEvent(event);
	update();
	//draw();
	// Need to render the frame at the end, or nothing actually gets shown on the screen !
	window.renderFrame();
	}
}


void centerCameraPosition(){
	
	float minX = triangles.at(0).vertices[0].x;
	float maxX = triangles.at(0).vertices[0].x;
	float minY = triangles.at(0).vertices[0].y;
	float maxY = triangles.at(0).vertices[0].y;
	float minZ = triangles.at(0).vertices[0].z;
	float maxZ = triangles.at(0).vertices[0].z;
	float wx;
	float wy;
	float wz;
	
	//cout << maxX << " " << minX << " " << maxY << " " << minY << " " << endl;
	
	for(int i=0; i<(int)triangles.size(); i++){
		for(int j=0; j<3; j++){
			wx = triangles.at(i).vertices[j].x;
			wy = triangles.at(i).vertices[j].y;
			wz = triangles.at(i).vertices[j].z;
			if (wx <= minX) minX = wx;
			if (wx >= maxX) maxX = wx;
			if (wy <= minY) minY = wy;
			if (wy >= maxY) maxY = wy;
			if (wz <= minZ) minZ = wz;
			if (wz >= maxZ) maxZ = wz;
		}
	}
	
	cout << minZ << " " << maxZ << endl; 
	centerOfWorld = (maxZ + minZ)/2;
	
	cout << "z world center " << centerOfWorld << endl;
	
	//cout << maxX << " " << minX << " " << maxY << " " << minY << " " << endl;
	
	cameraPosition = {(maxX + minX)/2, (maxY + minY)/2, cameraStepBack};
	cout << "camera: "<< cameraPosition.x << " "<< cameraPosition.y<< " "<< cameraPosition.z <<endl;
			
}


bool loadObjFile(string filepath){
	ifstream file;
	file.open(filepath);
	
	string o = "o";
	string usemtl = "usemtl";
	string v = "v";
	string f = "f";
	
	string objectName;
	string colorName;
	
	if (file.is_open()) {
		string line;
		char delim = ' ';
		
		while (getline(file, line)) {
			//printf("%s", line.c_str());
			//cout << endl;
			string *tokens = split(line, delim);
/* 			int numberOfTokens = count(line.begin(), line.end(), delim) + 1;
			for (int i=0; i<numberOfTokens; i++){
				cout << tokens[i] << endl;
			} */
			
			// o line
			if (o.compare(tokens[0]) == 0){
				objectName = tokens[1];
			} 
			
			// usemtl line
			else if (usemtl.compare(tokens[0]) == 0){
				colorName = tokens[1];
			}
			
			// v line
			else if (v.compare(tokens[0]) == 0){
				float v1 = stof(tokens[1]);
				float v2 = stof(tokens[2]);
				float v3 = stof(tokens[3]);
				vec3 vertex = vec3(v1, v2, v3);
				vertices.push_back(vertex);
			}
			
			// f line
			else if (f.compare(tokens[0]) == 0) {
				vec3 faceVertices[3];
				for (int i = 0; i < 3; i++) {
					string *faceValues = split (tokens[i+1], '/');
					int vertexIndex = (stoi(faceValues[0])) - 1;
					// int textureNumber = stoi (faceValues [i]) // question mark
					faceVertices[i] = vertices.at(vertexIndex);
				}
				
				//for (int i=0; i<3; i++){
					//cout << "first vertex in a triangle: " <<faceVertices[0].x << " " <<faceVertices[0].y << " " <<faceVertices[0].z << endl;
				//}
				
				int paletteIndex=0;
				while (colorName.compare(palette.at(paletteIndex).name) != 0 && paletteIndex<(int)palette.size()){
					paletteIndex++;
				}
				
				Colour color = palette.at(paletteIndex);
				//cout << color.name << endl;
				
				
				ModelTriangle triangle = ModelTriangle(faceVertices[0], faceVertices[1], 
														faceVertices[2], color);
				
				triangles.push_back(triangle);
			}
			
		}
		
		file.close();
	} else {
		printf("Loading error!\n");
		return false;
	}
	
	return true;
}

bool loadMtlFile(string filepath){
	ifstream file;
	file.open(filepath);
	
	if (file.is_open()) {
		string line;
		char delim = ' ';
		string newmtl = "newmtl";
		string kd = "Kd";
		string colorName;

		while (getline(file, line)) {
			//printf("%s", line.c_str());
			//cout << endl;
			string *tokens = split(line, delim);
			//int numberOfTokens = count(line.begin(), line.end(), delim) + 1;
			//cout << tokens << endl;
			
			// newmtl line
			if (newmtl.compare(tokens[0]) == 0){
				//cout << "MTL match" << endl;
				colorName = tokens[1];
			}
			// Kd line
			else if (kd.compare(tokens[0]) == 0){
				//cout << "KD match" << endl;
				int r = std::round(stof(tokens[1]) * 255);
				int g = std::round(stof(tokens[2]) * 255);
				int b = std::round(stof(tokens[3]) * 255);
				
				palette.push_back(Colour(colorName, r, g, b));
			}
			
		}
		file.close();
	} else {
		printf("Loading error!\n");
		return false;
	}
	
	return true;
}	

void pointWorldToCameraSpace(){
	// loop through all triangles in the world
	for(int i=0; i<(int)triangles.size(); i++){

/* 		cout << "firt vertex of every stored triangles in triangles " 
		<< triangles.at(i).vertices[0].x << " "
		<< triangles.at(i).vertices[0].y << " "
		<< triangles.at(i).vertices[0].z << endl; */
		//cout<<"triangle: "<< i+1 <<endl;
		
		Colour colour = triangles.at(i).colour;
		uint32_t pixel = (255<<24) + (int(colour.red)<<16) + (int(colour.green)<<8) + int(colour.blue);
		
		// loop through all vertices in the world triangle																
		for(int j=0; j<3; j++){
			pointWorldSpace = vec3(triangles.at(i).vertices[j].x,
							       triangles.at(i).vertices[j].y,
							       triangles.at(i).vertices[j].z);
			
			pointCameraSpace = pointWorldSpace - cameraPosition;
			
/* 			cout << "world point " << j+1 << ": "<< pointWorldSpace.x << " "
				<< pointWorldSpace.y<< " "<< pointWorldSpace.z <<endl; */
			
			
/* 			cout << "camera point "<< j+1 << ": "<<pointCameraSpace.x << " " 
				<< pointCameraSpace.y << " " << pointCameraSpace.z << endl; */
			
			// perspective projection
			float x = pointCameraSpace.x;
			float y = pointCameraSpace.y;
			float z = pointCameraSpace.z;
			
			float cx = focalLength * (x / z);
			float cy = focalLength * (y / z);
			float cz = focalLength;
			
			//cout << x << " " << y << " " << z << " " << cx << endl;
			
			// calculate window coordinates and scale
			pointCanvasSpace.x = round(cx * scale + WIDTH/2); 
			pointCanvasSpace.y = round(cy * scale + WIDTH/2);
			pointCanvasSpace.z = round(cz * scale + WIDTH/2); 
			
			window.setPixelColour(pointCanvasSpace.x, pointCanvasSpace.y, pixel);
			vcounter++;
			cout << "vertex " << vcounter << " drawn" << endl;
		}
	}
	//cout << (int)triangles.size();
}

void draw()
{
  window.clearPixels();
  for(int y=0; y<window.height ;y++) {
    for(int x=0; x<window.width ;x++) {
      float red = rand () % 255;
      float green = 0.0;
      float blue = 0.0;
      uint32_t colour = (255<<24) + (int(red)<<16) + (int(green)<<8) + int(blue);
      window.setPixelColour(x, y, colour);
    }
  }
}

void drawSquare()
{
  window.clearPixels();
  for(int y=0; y<50 ;y++) {
    for(int x=0; x<50 ;x++) {
      float red = rand() % 255;
      float green = rand() % 255;
      float blue = rand() % 255;
      uint32_t colour = (255<<24) + (int(red)<<16) + (int(green)<<8) + int(blue);
      window.setPixelColour(x, y, colour);
    }
  }
}

void update()
{
  // Function for performing animation (shifting artifacts or moving the camera)
}

void handleEvent(SDL_Event event)
{
  if(event.type == SDL_KEYDOWN) {
    if(event.key.keysym.sym == SDLK_LEFT) cout << "LEFT" << endl;
    else if(event.key.keysym.sym == SDLK_RIGHT) cout << "RIGHT" << endl;
    else if(event.key.keysym.sym == SDLK_UP) cout << "UP" << endl;
    else if(event.key.keysym.sym == SDLK_DOWN) cout << "DOWN" << endl;
  }
  else if(event.type == SDL_MOUSEBUTTONDOWN) {
		cout << "Mouse clicked" << endl;
		drawSquare();
  }
}

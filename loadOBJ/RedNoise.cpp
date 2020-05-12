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

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
vector<Colour> palette;
vector<vec3> vertices;
vector<ModelTriangle> triangles;

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
	
/* 	for (Colour c : palette){
		cout << c << endl; 
	} */
	
	SDL_Event event;
	while(true)
	{
	// We MUST poll for events - otherwise the window will freeze !
	if(window.pollForInputEvents(&event)) handleEvent(event);
	update();
	draw();
	// Need to render the frame at the end, or nothing actually gets shown on the screen !
	window.renderFrame();
	}
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
				
/* 				for (int i=0; i<3; i++){
					cout << faceVertices[i].x << " " <<faceVertices[i].y << " " <<faceVertices[i].z << endl;
				} */
				
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

void draw()
{
  window.clearPixels();
  for(int y=0; y<window.height ;y++) {
    for(int x=0; x<window.width ;x++) {
      float red = rand() % 255;
      float green = 0.0;
      float blue = 0.0;
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
  else if(event.type == SDL_MOUSEBUTTONDOWN) cout << "Mouse clicked" << endl;
}

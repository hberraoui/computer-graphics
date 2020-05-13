#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <PixelMap.h>
#include <fstream>
#include <vector>
#include <fstream>
#include <string.h>

using namespace std;
using namespace glm;

#define WIDTH 640	
#define HEIGHT 480

// IMPORTED
#define WHITE Colour(255,255,255)
#define RED Colour(255,0,0)
#define GREEN Colour(0,255,0)
#define BLUE Colour(0,0,255)
#define BLACK Colour(0,0,0)
// --------------------

string cornellBoxMtlPath = "../cornell-box/cornell-box.mtl";
string cornellBoxObjPath = "../cornell-box/cornell-box.obj";
string hackspaceLogoMtlPath = "../hackspace_logo/materials.mtl";
string hackspaceLogoObjPath = "../hackspace_logo/logo.obj";

void draw();
void update();
void handleEvent(SDL_Event event);
bool loadMtlFile(string filepath);
bool loadObjFile(string filepath);
void centerCameraPosition();
void transformVertexCoordinatesToCanvas();
void drawStrokedTriangles();
void drawLine(CanvasPoint from, CanvasPoint to, Colour colour);
int calcSteps(CanvasPoint from, CanvasPoint to);
int calcSteps(float fromX, float fromY, float toX, float toY);

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
vector<Colour> palette;
vector<vec3> textureVertices;
vector<vec2> vertexTextureIndexes;
vector<vec3> vertices;
vector<ModelTriangle> triangles;
vector<CanvasTriangle> displayTriangles;

// LOGO
// float cameraStepBack = 200;
// int scale = 1;

// CORNELL BOX
float cameraStepBack = 10;
int scale = 120;

float focalLength;
vec3 pointCameraSpace;
vec3 pointCanvasSpace;

vec3 pointWorldSpace;

float centerOfWorld;

//int vcounter = 0;
	
vec3 cameraPosition;




// IMPORTED STUFF ----------------------------------------------------
float interpolationStep(float start, float from, float to, int steps);
std::vector<float> interpolate(float from, float to, int steps);
std::vector<vec3> interpolate(vec3 from, vec3 to, int steps);
std::vector<CanvasPoint> interpolate(CanvasPoint from, CanvasPoint to, int steps);
int calcSteps(float fromX, float fromY, float toX, float toY);
int calcSteps(CanvasPoint from, CanvasPoint to);
int calcSteps(vec3 from, vec3 to);
float getXFromY(float y, CanvasPoint from, CanvasPoint to);
bool drawPixelMap(PixelMap img, int startX, int startY);
bool drawPixelMap(PixelMap img);
PixelMap loadPixelMap(string fn);
void saveImage();
void textureMappingTask();
void drawImage();
void drawRandomFilledTriangle();
void drawFilledTriangle(CanvasTriangle t, PixelMap img);
void drawFilledTriangle(CanvasTriangle triangle, Colour colour);
void drawFilledTriangle(CanvasTriangle triangle);
void drawRandomStrokedTriangle();
void drawStrokedTriangle(CanvasTriangle triangle, Colour colour);
void drawStrokedTriangle(CanvasTriangle triangle);
void drawLine(CanvasPoint from, CanvasPoint to, Colour colour);
void drawLine(TexturePoint from, TexturePoint to, Colour colour);
uint32_t vec3ToPackedInt(vec3 pixel);
vec3 packedIntToVec3(uint32_t colour);
void rainbowInterpolation();
void greyscaleInterpolation();
void drawRedNoise();

// --------------------------------------------------------------------------


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
	
	//loadMtlFile(hackspaceLogoMtlPath);
	//loadObjFile(hackspaceLogoObjPath);
	// we have world triangles
	
	centerCameraPosition();
	focalLength = abs((cameraPosition.z - centerOfWorld)/2);
	
	cout << "f: " << focalLength << endl;
	
	transformVertexCoordinatesToCanvas();
		
		
	
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
	
	cout << maxX << " " << minX << " " << maxY << " " << minY << " " << maxZ << " " << minZ << endl;
	
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
	string vt = "vt";
	string mtllib = "mtllib";
	
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
			
			// v or vt line
			else if (v.compare(tokens[0]) == 0 || vt.compare(tokens[0]) == 0){
				float v1 = stof(tokens[1]);
				float v2 = stof(tokens[2]);
				float v3 = stof(tokens[3]);
				vec3 vertex = vec3(v1, v2, v3);
				if (v.compare(tokens[0]) == 0)
					vertices.push_back(vertex);
				else
					textureVertices.push_back(vertex);
			}
			
			// f line
			else if (f.compare(tokens[0]) == 0) {
				vec3 faceVertices[3];
				
				for (int i = 0; i < 3; i++) {
					string *faceValues = split (tokens[i+1], '/');
					
					// check size of faceValues
					//cout << "fv size: " << (int)faceValues->size() << endl;
					//cout << "fv1 value: " << faceValues[1] << endl;
					
					int vertexIndex = (stoi(faceValues[0])) - 1;
					
					if (faceValues[1].compare("") != 0) {
						int textureIndex = (stoi(faceValues[1])) - 1; // question mark
						cout << "tex num" << textureIndex << endl;
						vertexTextureIndexes.push_back({vertexIndex, textureIndex});
						
					}
					
					faceVertices[i] = vertices.at(vertexIndex);
					
				}
				
				//for (int i=0; i<3; i++){
					//cout << "first vertex in a triangle: " <<faceVertices[0].x << " " <<faceVertices[0].y << " " <<faceVertices[0].z << endl;
				//}
				
				Colour colour = RED;
				
				if ((int)palette.size() > 0){
				
					int paletteIndex=0;
					while (colorName.compare(palette.at(paletteIndex).name) != 0 && paletteIndex<(int)palette.size()){
						paletteIndex++;
					}
					
					colour = palette.at(paletteIndex);
					//cout << colour.name << endl;
				}
				
				
				ModelTriangle triangle = ModelTriangle(faceVertices[0], faceVertices[1], 
														faceVertices[2], colour);
				
				triangles.push_back(triangle);
			} 
			
			else if (mtllib.compare(tokens[0]) == 0) {
				// could get the mtllibrary name
			}
			
			else {
				if (line != ""){ 
					cout << "undefined line in OBJ" << endl; 
					cout << line << endl;
				}
			}
			
		}
		
		cout << "I am finished with obj" << endl; 
		
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
			} else {
				if (line != ""){
					cout << "undefined line in MTL" << endl;
					cout << line << endl;
				}
			}
			
		}
		
		cout << "I am finished with mtl" << endl; 
		
		file.close();
	} else {
		printf("Loading error!\n");
		return false;
	}
	
	return true;
}	

void transformVertexCoordinatesToCanvas(){
	displayTriangles.clear();
	// loop through all triangles in the world
	for(int i=0; i<(int)triangles.size(); i++){

/* 		cout << "firt vertex of every stored triangles in triangles " 
		<< triangles.at(i).vertices[0].x << " "
		<< triangles.at(i).vertices[0].y << " "
		<< triangles.at(i).vertices[0].z << endl; */
		//cout<<"triangle: "<< i+1 <<endl;
		
		Colour colour = triangles.at(i).colour;
		//uint32_t pixel = (255<<24) + (int(colour.red)<<16) + (int(colour.green)<<8) + int(colour.blue);
		
		CanvasPoint displayVertices[3];
		
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
			//float cz = focalLength;
			
			//cout << x << " " << y << " " << z << " " << cx << endl;
			
			// calculate window coordinates and scale
			pointCanvasSpace.x = round(WIDTH/2 - cx * scale); 
			pointCanvasSpace.y = round(cy * scale + HEIGHT/2);
			//pointCanvasSpace.z = round(cz * scale + WIDTH/2); 
			
			
			// Draw
			//window.setPixelColour(pointCanvasSpace.x, pointCanvasSpace.y, pixel);
			
			//vcounter++;
			//cout << "vertex " << vcounter << " drawn" << endl;
			
			displayVertices[j] = CanvasPoint(pointCanvasSpace.x, pointCanvasSpace.y);
		}
		
		CanvasTriangle triangle = CanvasTriangle(displayVertices[0], displayVertices[1], 
											   displayVertices[2], colour);
											   
		displayTriangles.push_back(triangle);	
		
		
	}
	//cout << (int)triangles.size();
}


void update()
{
  // Function for performing animation (shifting artifacts or moving the camera)
  transformVertexCoordinatesToCanvas();
}

int triangleStrokeCounter = 0;
int triangleFillCounter = 0;

void drawAllFilledTriangles(){
	window.clearPixels();
	
	while (triangleFillCounter < (int)displayTriangles.size()){
			drawFilledTriangle(displayTriangles.at(triangleFillCounter));
			triangleFillCounter++;
			//cout << "filled a triangle " << triangleFillCounter << endl;
	}
	
	triangleFillCounter = 0;
}

void drawAllStrokedTriangles(){
	window.clearPixels();
	
	while (triangleStrokeCounter < (int)displayTriangles.size()){
			drawStrokedTriangle(displayTriangles.at(triangleStrokeCounter));
			triangleStrokeCounter++;
			//cout << "stroked a triangle " << triangleStrokeCounter << endl;
	}
	
	triangleStrokeCounter = 0;
}


void handleEvent(SDL_Event event)
{
  if(event.type == SDL_KEYDOWN) {
    if(event.key.keysym.sym == SDLK_LEFT) cout << "LEFT" << endl;
    else if(event.key.keysym.sym == SDLK_RIGHT) cameraPosition.x += 0.2;
	
	
    else if(event.key.keysym.sym == SDLK_UP) {
		cout << "UP pressed" << endl;
		drawAllFilledTriangles();

	}
    else if(event.key.keysym.sym == SDLK_DOWN) cout << "DOWN" << endl;
	}
	
	
	else if(event.type == SDL_MOUSEBUTTONDOWN) {
		cout << "Mouse clicked" << endl;
		drawAllStrokedTriangles();
  }
}

// IMPORTED STUFF ---------------------------------------------------------------------
/////////////////
// STRUCTURE
////////////////


/////////////////
// FUNCTIONS USED A LOT
////////////////

int calcSteps(float fromX, float fromY, float toX, float toY)
{
    float xDiff = toX - fromX;
    float yDiff = toY - fromY;
    return std::floor(std::max(abs(xDiff), abs(yDiff)));
}

int calcSteps(CanvasPoint from, CanvasPoint to)
{
    return calcSteps(from.x, from.y, to.x, to.y);
}

int calcSteps(vec3 from, vec3 to)
{
    return calcSteps(from.x, from.y, to.x, to.y);
}

int calcSteps(TexturePoint from, TexturePoint to)
{
    return calcSteps(from.x, from.y, to.x, to.y);
}

float getXFromY(float yTarget, CanvasPoint from, CanvasPoint to)
{
    float x = -1;
    int steps = calcSteps(from,to);
    std::vector<CanvasPoint> points = interpolate(from, to, steps);
    for (int i = 0; i < steps; i++) {
        if (std::floor(points.at(i).y) == std::floor(yTarget)) {
            x = points.at(i).x;
            break;
        }
    }
    return x;
}

float getXFromY(float yTarget, TexturePoint from, TexturePoint to)
{
    return getXFromY(yTarget,CanvasPoint(from.x,from.y),CanvasPoint(to.x,to.y));
}

uint32_t vec3ToPackedInt(vec3 pixel)
{
    return (255<<24) + (int(pixel.x)<<16) + (int(pixel.y)<<8) + int(pixel.z);
}

vec3 packedIntToVec3(uint32_t colour)
{
    vec3 result( (colour<<8)>>24, (colour<<16)>>24, (colour<<24)>>24 );
    return result;
}

/////////////////
// 1D TASKS
////////////////

// 1D task 1
void drawRedNoise()
{
    window.clearPixels();
    for(int y=0; y < window.height; y++) {
        for(int x=0; x < window.width; x++) {
            float red = rand() % 255;
            float green = 0.0;
            float blue = 0.0;
            uint32_t colour = (255<<24) + (int(red)<<16) + (int(green)<<8) + int(blue);
            window.setPixelColour(x, y, colour);
        }
    }
}

// 1D task 2
float interpolationStep(float lastCalculated, float from, float to, int steps)
{
    return lastCalculated + (to - from) / steps;
}

std::vector<float> interpolate(float from, float to, int steps)
{
    std::vector<float> v = { from };
    for (int i = 0; i < steps; i++) {
        v.push_back(interpolationStep(v.back(), from, to, steps));
    }
    return v;
}

std::vector<CanvasPoint> interpolate(CanvasPoint from, CanvasPoint to, int steps)
{
    std::vector<CanvasPoint> v = { from };
    for (int i = 0; i < steps; i++) {
        CanvasPoint newCV(interpolationStep(v.back().x, from.x, to.x, steps),
                          interpolationStep(v.back().y, from.y, to.y, steps));
        v.push_back(newCV);
    }
    return v;
}

// 1D task 3
void greyscaleInterpolation()
{
    // Drawing a one dimension greyscale interpolation
    window.clearPixels();
    std::vector<float> v;
    v = interpolate(0, 255, window.width);
    for(int y = 0; y < window.height; y++) {
        for(int x = 0; x < window.width; x++) {
            vec3 c(v.at(x), v.at(x), v.at(x));
            window.setPixelColour(x, y, vec3ToPackedInt(c));
        }
    }
}

// 1D task 4
std::vector<vec3> interpolate(vec3 from, vec3 to, int steps)
{
    std::vector<vec3> v = { from };
    for (int i = 0; i < steps; i++) {
        vec3 newvec(interpolationStep(v.back().x, from.x, to.x, steps),
                    interpolationStep(v.back().y, from.y, to.y, steps),
                    interpolationStep(v.back().z, from.z, to.z, steps));
        v.push_back(newvec);
    }
    return v;
}

// 1D task 5
void rainbowInterpolation()
{
    // Draw a two dimension colour interpolation
    window.clearPixels();
    vec3 redPixel(255,0,0);
    vec3 bluePixel(0,0,255);
    vec3 yellowPixel(255,255,0);
    vec3 greenPixel(0,255,0);

    window.setPixelColour(0, window.height-1, vec3ToPackedInt(yellowPixel));
    window.setPixelColour(window.width-1, window.height-1, vec3ToPackedInt(greenPixel));

    std::vector<vec3> v;
    v = interpolate(redPixel, yellowPixel, window.height);
    for(int y = 0; y < window.height; y++) {
        window.setPixelColour(0, y, vec3ToPackedInt(v.at(y)));
    }

    v = interpolate(bluePixel, greenPixel, window.height);
    for(int y=0; y < window.height; y++) {
        window.setPixelColour(window.width-1, y, vec3ToPackedInt(v.at(y)));
    }

    for(int y=0; y < window.height; y++) {
        v = interpolate(packedIntToVec3(window.getPixelColour(0,y)),
                                        packedIntToVec3(window.getPixelColour(window.width-1,y)),
                                        window.width);
        for(int x=0; x < window.width; x++) {
            window.setPixelColour(x, y, vec3ToPackedInt(v.at(x)));
        }
    }
}

/////////////////
// 2D TASKS
////////////////

// 2D task 1
void drawLine(CanvasPoint from, CanvasPoint to, Colour colour)
{
    int steps = calcSteps(from,to);
    std::vector<CanvasPoint> points = interpolate(from, to, steps);
    for (int i = 0; i < steps; i++) {
        window.setPixelColour(std::floor(points.at(i).x), std::floor(points.at(i).y), colour.toPackedInt());
    }
}

void drawLine(TexturePoint from, TexturePoint to, Colour colour)
{
    drawLine(CanvasPoint(from.x, from.y), CanvasPoint(to.x, to.y), colour);
}

// 2D task 2
void drawRandomStrokedTriangle()
{
    CanvasTriangle t(CanvasPoint(rand() % window.width, rand() % window.height),
                     CanvasPoint(rand() % window.width, rand() % window.height),
                     CanvasPoint(rand() % window.width, rand() % window.height),
                     Colour(rand() % 255, rand() % 255, rand() % 255));
    drawStrokedTriangle(t);
}

void drawStrokedTriangle(CanvasTriangle triangle, Colour colour)
{
    drawLine(triangle.vertices[0], triangle.vertices[1], colour);
    drawLine(triangle.vertices[0], triangle.vertices[2], colour);
    drawLine(triangle.vertices[1], triangle.vertices[2], colour);
}

void drawStrokedTriangle(CanvasTriangle triangle)
{
    drawStrokedTriangle(triangle, triangle.colour);
}

// 2D task 3
void drawRandomFilledTriangle()
{
    CanvasTriangle t(CanvasPoint(rand() % window.width, rand() % window.height),
                     CanvasPoint(rand() % window.width, rand() % window.height),
                     CanvasPoint(rand() % window.width, rand() % window.height),
                     Colour(rand() % 255, rand() % 255, rand() % 255));
    drawFilledTriangle(t);
}
void drawFilledTriangle(CanvasTriangle t, Colour colour)
{
    // Sort vertices by vertical position (top to bottom)
    t.sortVertices();

	if (t.vertices[1].y != t.vertices[2].y) { 

		// Divide triangle into 2 "flat-bottomed" triangles
		float vy = t.vertices[1].y;
		float vx = getXFromY(vy, t.vertices[0], t.vertices[2]);
		CanvasPoint v(vx,vy);
		
		CanvasTriangle topT = CanvasTriangle(t.vertices[0], t.vertices[1], v);
		CanvasTriangle bottomT = CanvasTriangle(t.vertices[1], v, t.vertices[2]);

		// Fill top triangle (top-to-bottom, left-to-right)
		for (int y = topT.vertices[0].y; y < topT.vertices[2].y; ++y) {
			float x1 = getXFromY(y, topT.vertices[0], topT.vertices[1]);
			float x2 = getXFromY(y, topT.vertices[0], topT.vertices[2]);
			float startX = std::min(x1,x2);
			float endX = std::max(x1,x2);
			for (int x = startX; x <= endX; x++)
				window.setPixelColour(x, y, colour.toPackedInt());
		}

		// Fill bottom triangle (top-to-bottom, left-to-right)
		for (int y = bottomT.vertices[0].y; y < bottomT.vertices[2].y; ++y) {
			float x1 = getXFromY(y, bottomT.vertices[0], bottomT.vertices[2]);
			float x2 = getXFromY(y, bottomT.vertices[1], bottomT.vertices[2]);
			float startX = std::min(x1,x2);
			float endX = std::max(x1,x2);
			for (int x = startX; x <= endX; x++)
				window.setPixelColour(x, y, colour.toPackedInt());
		}
	
	} 
	else {
		for (int y = t.vertices[0].y; y < t.vertices[2].y; ++y) {
			float x1 = getXFromY(y, t.vertices[0], t.vertices[1]);
			float x2 = getXFromY(y, t.vertices[0], t.vertices[2]);
			float startX = std::min(x1,x2);
			float endX = std::max(x1,x2);
			for (int x = startX; x <= endX; x++)
				window.setPixelColour(x, y, colour.toPackedInt());
		}
	}

    // Test the fill is accurate by drawing the original triangle
    drawStrokedTriangle(t, t.colour);
}

void drawFilledTriangle(CanvasTriangle t)
{
    drawFilledTriangle(t, t.colour);
}

// 2D Task 4
bool drawPixelMap(PixelMap img, int startX, int startY)
{
    if (img.pixels.size() > 0) {
        for (int y = startY; y < img.height; y++) {
            for (int x = startX; x < img.width; x++) {
                uint32_t pixel = img.pixels.at(x + img.width*y);
                if (x < window.width && y < window.height) {
                    window.setPixelColour(x,y,pixel);
                }
            }
        }
        return true;
    }
    return false;
}

bool drawPixelMap(PixelMap img)
{
    return drawPixelMap(img, 0, 0);
}

PixelMap loadPixelMap(string fn)
{
    PixelMap img;

    std::ifstream ifs;
    ifs.open(fn, ios::binary);
    if (ifs.is_open()) {
        string line;

        // check for "magic number" to indicate correct encoding method (P6)
        getline(ifs,line);
        if (line != "P6") {
            cout << "ERROR: Unsupported PPM format. Closing file." << endl;
            ifs.close();
            return img;
        }

        // ignore comments
        getline(ifs,line);
        while (line[0] == '#') getline(ifs,line);

        // read image dimensions
        int width = std::stoi(line.substr(0, line.find(' ')));
        int height = std::stoi(line.substr(line.find(' '), line.length()));

        // read max colour value
        getline(ifs,line);
        int maxColourValue = std::stoi(line);

        // read the pixel data payload
        std::vector<uint32_t> texture = { };
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                char r, g, b;
                ifs.get(r);
                ifs.get(g);
                ifs.get(b);
                uint32_t pixel = (255<<24) + ((r % maxColourValue)<<16) + ((g % maxColourValue)<<8) + (b % maxColourValue);
                texture.push_back(pixel);
            }
        }

        ifs.close();

        // populate the PixelMap's fields
        img.width = width;
        img.height = height;
        img.pixels = texture;
    } else {
        cout << "ERROR: Cannot open file." << endl;
    }

    return img;
}

void drawImage()
{
    PixelMap img = loadPixelMap("texture.ppm");
    drawPixelMap(img);
}

// 2D task 5
/* void textureMappingTask()
{
    PixelMap img = loadPixelMap("texture.ppm");
    CanvasTriangle t(CanvasPoint(160, 10,TexturePoint(195,  5)),
                     CanvasPoint(300,230,TexturePoint(395,380)),
                     CanvasPoint( 10,150,TexturePoint( 65,330)));
    drawFilledTriangle(t, img);
    drawStrokedTriangle(t);
} */

std::vector<TexturePoint> interpolate(TexturePoint from, TexturePoint to, int steps)
{
    std::vector<TexturePoint> v = { from };
    for (int i = 0; i < steps; i++) {
        v.push_back(TexturePoint(interpolationStep(v.back().x, from.x, to.x, steps),
                                 interpolationStep(v.back().y, from.y, to.y, steps)));
    }
    return v;
}

void drawFilledTriangle(CanvasTriangle t, PixelMap img)
{
    // Sort vertices by vertical position (top to bottom)
    t.sortVertices();

    CanvasPoint top = t.vertices[0];
    CanvasPoint middle = t.vertices[1];
    CanvasPoint bottom = t.vertices[2];

    // Interpolate "top" and "bottom" to find "flat" division vertex
    float divVy = middle.y;
    float divVx = getXFromY(divVy, top, bottom);
    CanvasPoint divV(divVx,divVy);

    // Calculate division vertex's texture point
    int stepsTopToBottom = calcSteps(top, bottom);
    int stepsTopToDivV = calcSteps(top, divV);
    double proportion = (stepsTopToDivV * 1000) / stepsTopToBottom;
    proportion = proportion / 1000; // Use 1000 multiplier to avoid low-bit loss
    int stepsTexTopToBottom = calcSteps(top.texturePoint, bottom.texturePoint);
    int stepsTexTopToDivV = std::floor(stepsTexTopToBottom * proportion);
    std::vector<TexturePoint> texPoints = interpolate(top.texturePoint, bottom.texturePoint, stepsTexTopToBottom);
    divV.texturePoint = texPoints.at(stepsTexTopToDivV);

    // Divide triangle into 2 "flat" triangles
    CanvasTriangle topT = CanvasTriangle(top, middle, divV);
    CanvasTriangle bottomT = CanvasTriangle(middle, divV, bottom);

    // Tackle the top triangle first
    int numberOfRows;
    std::vector<CanvasPoint> canvasStartPoints;
    std::vector<CanvasPoint> canvasEndPoints;
    std::vector<TexturePoint> textureStartPoints;
    std::vector<TexturePoint> textureEndPoints;
    
    // Select biggest number of steps
    numberOfRows = std::max((topT.vertices[1].y - topT.vertices[0].y),
                            (topT.vertices[2].y - topT.vertices[0].y));
    
    // Interpolate start and end points in canvas space
    canvasStartPoints = interpolate(topT.vertices[0], topT.vertices[1], numberOfRows);
    canvasEndPoints = interpolate(topT.vertices[0], topT.vertices[2], numberOfRows);
    
    // Interpolate start and end points in texture space
    textureStartPoints = interpolate(topT.vertices[0].texturePoint,
                                     topT.vertices[1].texturePoint,
                                     numberOfRows);
    textureEndPoints = interpolate(topT.vertices[0].texturePoint,
                                   topT.vertices[2].texturePoint,
                                   numberOfRows);

    for (int row = 0; row < numberOfRows; row++) {
        CanvasPoint startPoint = canvasStartPoints.at(row);
        CanvasPoint endPoint = canvasEndPoints.at(row);
        int canvasRowLength = std::round(endPoint.x - startPoint.x);
        std::vector<TexturePoint> textureRow = interpolate(textureStartPoints.at(row),
                                                           textureEndPoints.at(row),
                                                           canvasRowLength);

        for (int column = 0; column < (int) textureRow.size(); column++) {
            int x = startPoint.x + column;
            int y = topT.vertices[0].y + row;
            TexturePoint texP = textureRow.at(column);
            uint32_t pixel = img.pixels.at(std::round(texP.x) + (img.width * std::round(texP.y)));
            window.setPixelColour(x, y, pixel);
        }
    }
    
    // Tackle the bottom triangle next, again by selecting biggest number of steps
    numberOfRows = std::max((bottomT.vertices[2].y - bottomT.vertices[0].y),
                            (bottomT.vertices[2].y - bottomT.vertices[1].y));
    
    // Interpolate start and end points in canvas space
    canvasStartPoints = interpolate(bottomT.vertices[0], bottomT.vertices[2], numberOfRows);
    canvasEndPoints = interpolate(bottomT.vertices[1], bottomT.vertices[2], numberOfRows);
    
    // Interpolate start and end points in texture space
    textureStartPoints = interpolate(bottomT.vertices[0].texturePoint,
                                     bottomT.vertices[2].texturePoint,
                                     numberOfRows);
    textureEndPoints = interpolate(bottomT.vertices[1].texturePoint,
                                   bottomT.vertices[2].texturePoint,
                                   numberOfRows);

    for (int row = 0; row < numberOfRows; row++) {
        CanvasPoint startPoint = canvasStartPoints.at(row);
        CanvasPoint endPoint = canvasEndPoints.at(row);
        int canvasRowLength = std::round(endPoint.x - startPoint.x);
        std::vector<TexturePoint> textureRow = interpolate(textureStartPoints.at(row),
                                                           textureEndPoints.at(row),
                                                           canvasRowLength);

        for (int column = 0; column < (int) textureRow.size(); column++) {
            int x = startPoint.x + column;
            int y = bottomT.vertices[0].y + row;
            TexturePoint texP = textureRow.at(column);
            uint32_t pixel = img.pixels.at(std::round(texP.x) + (img.width * std::round(texP.y)));
            window.setPixelColour(x, y, pixel);
        }
    }
}

// Saving to PPM
void saveAreaAsImage(int width, int height);
void saveImage()
{
    saveAreaAsImage(window.width/2, window.height);
}

void saveAreaAsImage(int width, int height)
{
    PixelMap img;

    img.width = width;
    img.height = height;
    img.pixels = {};
    
    for (int row = 0; row < img.height; row++) {
        for (int column = 0; column < img.width; column++) {
            img.pixels.push_back(window.getPixelColour(column, row));
        }
    }
    
    ofstream myfile ("output.ppm", ios::binary);
    if (myfile.is_open()) {
        myfile << "P6\n";
        myfile << img.width << " " << img.height << endl;
        myfile << "255" << endl;
        for (int i = 0; i < (int) img.pixels.size(); i++) {
            char r = (img.pixels.at(i) << 8)       >> 24;
            char g = (img.pixels.at(i) << 16)  >> 24;
            char b = (img.pixels.at(i) << 24) >> 24;
            myfile << r;
            myfile << g;
            myfile << b;
        }
    }
    
    cout << "SAVED WINDOW AS PPM IMAGE FILE \"output.ppm\"" << endl;
}

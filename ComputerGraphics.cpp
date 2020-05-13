#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <PixelMap.h>
#include <fstream>
#include <vector>
#include <string.h>
#include <RayTriangleIntersection.h>

using namespace std;
using namespace glm;

void draw();
void update();
void handleEvent(SDL_Event event);
bool loadMtlFile(string filepath);
bool loadObjFile(string filepath);
void centerCameraPosition();
RayTriangleIntersection getClosestIntersection(int x, int y);
void renderWireframeCanvas();
void rasteriseCanvas();
void raytraceCanvas();
void redrawCanvas();
void changeRenderMode(int mode);
void transformVertexCoordinatesToCanvas();
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
void saveImage(int width, int height);
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
void drawPixel(int x, int y, uint32_t pixel);
uint32_t vec3ToPackedInt(vec3 pixel);
vec3 packedIntToVec3(uint32_t colour);
void rainbowInterpolation();
void greyscaleInterpolation();
void drawRedNoise();

#define WIDTH 640    
#define HEIGHT 480

string cornellBoxMtlPath = "./cornell-box/cornell-box.mtl";
string cornellBoxObjPath = "./cornell-box/cornell-box.obj";
string hackspaceLogoMtlPath = "./hackspace-logo/materials.mtl";
string hackspaceLogoObjPath = "./hackspace-logo/logo.obj";

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
vector<Colour> palette;
vector<vec3> vertices;
vector<ModelTriangle> triangles;
vector<CanvasTriangle> displayTriangles;

// LOGO
// float cameraStepBack = 200;
// int scale = 1;

// CORNELL BOX
float cameraStepBack = 40;
int scale = 120;

float focalLength;
vec3 pointCameraSpace;
vec3 pointCanvasSpace;
vec3 pointWorldSpace;
float centerOfWorld;
//int vcounter = 0;
vec3 cameraPosition;

enum renderModes {
    WIREFRAME,
    RASTER,
    RAYTRACE
};

int renderMode = RAYTRACE;

void redrawCanvas()
{
    switch (renderMode) {
        case WIREFRAME:
            renderWireframeCanvas();
            break;
        case RASTER:
            rasteriseCanvas();
            break;
        default:
        case RAYTRACE:
            raytraceCanvas();
            break;
    }
}

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

    centerCameraPosition();
    renderMode = WIREFRAME;
    redrawCanvas();
    
    SDL_Event event;
    while(true)
    {
        // We MUST poll for events - otherwise the window will freeze !
        if(window.pollForInputEvents(&event)) handleEvent(event);
        update();
        // draw();
        // Need to render the frame at the end, or nothing actually gets shown on the screen !
        window.renderFrame();
    }
}


void centerCameraPosition()
{
    float minX = triangles.at(0).vertices[0].x;
    float maxX = triangles.at(0).vertices[0].x;
    float minY = triangles.at(0).vertices[0].y;
    float maxY = triangles.at(0).vertices[0].y;
    float minZ = triangles.at(0).vertices[0].z;
    float maxZ = triangles.at(0).vertices[0].z;
    float wx;
    float wy;
    float wz;

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
    // cout << maxX << " " << minX << " " << maxY << " " << minY << " " << maxZ << " " << minZ << endl;
 
    centerOfWorld = (maxZ + minZ)/2;
    // cout << "z world center " << centerOfWorld << endl;
    
    cameraPosition = {(maxX + minX)/2, (maxY + minY)/2, cameraStepBack};
    // cout << "camera: " << cameraPosition.x << " "<< cameraPosition.y<< " "<< cameraPosition.z <<endl;

    focalLength = abs((cameraPosition.z - centerOfWorld)/2);
    // cout << "focal length: " << focalLength << endl;
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
            // cout << line.c_str() << endl;
            string *tokens = split(line, delim);
            
            /* int numberOfTokens = count(line.begin(), line.end(), delim) + 1;
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
                    
                    // check size of faceValues
                    // cout << "fv size: " << (int)faceValues->size() << endl;
                    // cout << "fv1 value: " << faceValues[1] << endl;
                    
                    int vertexIndex = (stoi(faceValues[0])) - 1;
                    
                    if (faceValues[1].compare("") != 0) {
                        int textureNumber = (stoi(faceValues[1])) - 1; // question mark
                        cout << "tex num" << textureNumber << endl;
                    }
                    
                    faceVertices[i] = vertices.at(vertexIndex);
                    
                }
                
                /* for (int i=0; i<3; i++){
                    cout << "first vertex in a triangle: " <<faceVertices[0].x << " " <<faceVertices[0].y << " " <<faceVertices[0].z << endl;
                } */
                
                Colour colour = RED;
                
                if ((int)palette.size() > 0){
                
                    int paletteIndex=0;
                    while (colorName.compare(palette.at(paletteIndex).name) != 0 && paletteIndex<(int)palette.size()){
                        paletteIndex++;
                    }
                    
                    colour = palette.at(paletteIndex);
                    // cout << colour.name << endl;
                }
                
                
                ModelTriangle triangle = ModelTriangle(faceVertices[0], faceVertices[1], 
                                                        faceVertices[2], colour);
                
                triangles.push_back(triangle);
            }
            
        }
        
        // cout << "I am finished with obj" << endl; 
        
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
            // cout << line.c_str() << endl;
            string *tokens = split(line, delim);
            // int numberOfTokens = count(line.begin(), line.end(), delim) + 1;
            // cout << tokens << endl;
            
            // newmtl line
            if (newmtl.compare(tokens[0]) == 0){
                // cout << "MTL match" << endl;
                colorName = tokens[1];
            }
            // Kd line
            else if (kd.compare(tokens[0]) == 0){
                // cout << "KD match" << endl;
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

void transformVertexCoordinatesToCanvas(){ // a.k.a Rasterisation
    displayTriangles.clear();
    // loop through all triangles in the world
    for(int i=0; i<(int)triangles.size(); i++){
        /* cout << "firt vertex of every stored triangles in triangles " 
             << triangles.at(i).vertices[0].x << " "
             << triangles.at(i).vertices[0].y << " "
             << triangles.at(i).vertices[0].z << endl; */
        // cout <<"triangle: "<< i+1 <<endl;
        
        Colour colour = triangles.at(i).colour;        
        CanvasPoint displayVertices[3];
        
        // loop through all vertices in the world triangle                                                                
        for(int j=0; j<3; j++){
            pointWorldSpace = vec3(triangles.at(i).vertices[j].x,
                                   triangles.at(i).vertices[j].y,
                                   triangles.at(i).vertices[j].z);
            
            pointCameraSpace = pointWorldSpace - cameraPosition;
            
            /* cout << "world point " << j+1 << ": " << pointWorldSpace.x << " "
                    << pointWorldSpace.y << " " << pointWorldSpace.z << endl; */
            
            
            /* cout << "camera point " << j+1 << ": " <<pointCameraSpace.x << " " 
                    << pointCameraSpace.y << " " << pointCameraSpace.z << endl; */
            
            // perspective projection
            float x = pointCameraSpace.x;
            float y = pointCameraSpace.y;
            float z = pointCameraSpace.z;
            
            float cx = focalLength * (x / z);
            float cy = focalLength * (y / z);
            // float cz = focalLength;
            
            // cout << x << " " << y << " " << z << " " << cx << endl;
            
            // calculate window coordinates and scale
            pointCanvasSpace.x = round(WIDTH/2 - cx * scale); 
            pointCanvasSpace.y = round(cy * scale + HEIGHT/2);
            // pointCanvasSpace.z = round(cz * scale + WIDTH/2); 

            /* // Draw vertex to make sure it is in the correct position
            drawPixel(pointCanvasSpace.x, pointCanvasSpace.y, colour.toPackedInt());
            vcounter++;
            cout << "vertex " << vcounter << " drawn" << endl; */
            
            displayVertices[j] = CanvasPoint(pointCanvasSpace.x, pointCanvasSpace.y);
        }
        
        CanvasTriangle t = CanvasTriangle(displayVertices[0],
                                          displayVertices[1], 
                                          displayVertices[2],
                                          colour);

        displayTriangles.push_back(t);
    }
    // cout << (int)triangles.size();
}

RayTriangleIntersection getClosestIntersection(int x, int y)
{
    RayTriangleIntersection closestIntersection;
    closestIntersection.distanceFromCamera = INFINITY;

    // A position along a ray can be represented as:
    // [1]      position = startPoint + scalar * direction
    //          r        = s          + t      * d
    
    // Our startPoint is the camera's position, the global variable vec3 cameraPosition
    // Calculate the ray's direction using the camera's position and the (x,y) of the pixel the ray intersects
    float xPos = (x - (window.width  / 2));
    float yPos = (y - (window.height / 2)) * -1;
    float zPos = (scale * focalLength)     * -1;
    vec3 rayDirection = normalize(vec3(xPos, yPos, zPos) - cameraPosition);
    
    // Given a triangular plane with vertices p0, p1, p2
    for (ModelTriangle triangle : triangles) {
        // Point r which intersects this triangular plane is defined as: 
        // [2]      r = p0 + u(p1 - p0) + v(p2 - p0)
        // Simplify [2] by referring to the differences between points as edges
        // [3]      r = p0 + ue0 + ve1
        vec3 e0 = triangle.vertices[1] - triangle.vertices[0];
        vec3 e1 = triangle.vertices[2] - triangle.vertices[0];
        
        // Combining [3] with [1], we get:
        // [4]      p0 + ue0 + ve1 = s + t * d
        // Rearrange [4] to get:
        // [5]      -t * d + ue0 + ve1 = s - p0
        // Represent [5] in matrix form:
        // [6]      [ -d.x e0.x e1.x ]   [ t ]   [ s.x - p0.x ]
        //          [ -d.y e0.y e1.y ] • [ u ] = [ s.y - p0.y ]
        //          [ -d.z e0.z e1.z ]   [ v ]   [ s.z - p0.z ]
        //          ^ "DEMatrix"                 ^ "SPVector"
        vec3 SPVector = cameraPosition - triangle.vertices[0];
        mat3 DEMatrix(-rayDirection, e0, e1);

        // Rearrange [6] to get:
        // [7]      [ t ]   [ -d.x e0.x e1.x ]^-1  [ s.x - p0.x ]
        //          [ u ] = [ -d.y e0.y e1.y ]  •  [ s.y - p0.y ]
        //          [ v ]   [ -d.z e0.z e1.z ]     [ s.z - p0.z ]    
        vec3 possibleSolution = glm::inverse(DEMatrix) * SPVector;
        
        // We have now calculated:
        //       t = the distance along the ray
        //       u = the u co-ordinate on the triangle's plane of the intersection point
        //       v = the v co-ordinate on the triangle's plane of the intersection point
        float distanceFromCamera = possibleSolution.x;
        float u = possibleSolution.y;
        float v = possibleSolution.z;

        // Before continuing, we must check the constraints on u,v are met:
        if (0.0f <= u && u <= 1.0f && 0.0f <= v && v <= 1.0f && (u + v) <= 1.0f) {
            // From here, we will need to transpose these (u,v) co-ordinates into model space
            // And then, like in rasterisation, from model space to canvas space
            float ndcX = (u + 0.5) / window.width;
            float ndcY = (v + 0.5) / window.height;
            float canvasX = 2 * (ndcX) - 1;
            float canvasY = 1 - 2 * (ndcY);
            
            vec3 intersectionPoint = vec3(canvasX, canvasY, (cameraPosition.z + distanceFromCamera));

            // If this intersection point is closer to the camera than the previous closest intersection
            if (closestIntersection.distanceFromCamera > distanceFromCamera) {
                // Then the new intersection we have found is the new closest intersection
                closestIntersection = RayTriangleIntersection(intersectionPoint, distanceFromCamera, triangle);
            }
        }
    }
    
    // After iterating over every triangle, we will now have found the intersection closest to the camera
    return closestIntersection;
}

void raytraceCanvas()
{
    // Iterate over every pixel in the canvas
    for (int y = 0; y < window.height; y++) {
        for (int x = 0; x < window.width; x++) {
            // Shoot a ray from the camera such that it intersects this pixel of the canvas
            RayTriangleIntersection closest = getClosestIntersection(x, y);

            Colour colour = BLACK;
            if (closest.distanceFromCamera != INFINITY) {
                colour = closest.intersectedTriangle.colour;
            }
            
            drawPixel(x, y, colour.toPackedInt());
        }
    }
}

void renderWireframeCanvas()
{
    window.clearPixels();
    transformVertexCoordinatesToCanvas();
    for (CanvasTriangle triangle : displayTriangles) {
        drawStrokedTriangle(triangle);
    }
}

void rasteriseCanvas()
{
    window.clearPixels();
    transformVertexCoordinatesToCanvas();
    for (CanvasTriangle triangle : displayTriangles) {
        drawFilledTriangle(triangle);
    }
}

void update()
{
    // Function for performing animation (shifting artifacts or moving the camera)
    // WIREFRAME:
    // renderWireframeCanvas();
    
    // RASTERISE:
    // rasteriseCanvas();
    
    // RAYTRACE:
    // raytraceCanvas();
}

// CONTROLS:
// ← Left arrow key: Shift camera position to the left
// → Right arrow key: Shift camera position to the right
// w Key: change to wireframe mode
// r Key: change to raster mode
// t Key: change to raytrace mode
// c Key: clear the window

float cameraSpeed = 0.2;

void changeRenderMode(int mode)
{
    renderMode = mode;
    redrawCanvas();
}

void handleEvent(SDL_Event event)
{
    if(event.type == SDL_KEYDOWN) {
        if(event.key.keysym.sym == SDLK_LEFT) {
            cameraPosition.x -= cameraSpeed;
            redrawCanvas();
            cout << "LEFT: Camera shifted left." << endl;
        } else if(event.key.keysym.sym == SDLK_RIGHT) {
            cameraPosition.x += cameraSpeed;
            redrawCanvas();
            cout << "RIGHT: Camera shifted right." << endl;
        } else if(event.key.keysym.sym == SDLK_UP) {
            cameraPosition.y -= cameraSpeed;
            redrawCanvas();
            cout << "UP: Camera shifted up." << endl;
        } else if(event.key.keysym.sym == SDLK_DOWN) {
            cameraPosition.y += cameraSpeed;
            redrawCanvas();
            cout << "DOWN: Camera shifted down." << endl;
        } else if(event.key.keysym.sym == SDLK_w) {
            changeRenderMode(WIREFRAME);
            cout << "w KEY: Switched to WIREFRAME MODE." << endl;
        } else if(event.key.keysym.sym == SDLK_r) {
            changeRenderMode(RASTER);
            cout << "r KEY: Switched to RASTER MODE." << endl;
        } else if(event.key.keysym.sym == SDLK_t) {
            changeRenderMode(RAYTRACE);
            cout << "r KEY: Switched to RAYTRACE MODE." << endl;
        } else if(event.key.keysym.sym == SDLK_c) {
            window.clearPixels();
            cout << "c KEY: Window cleared." << endl;
        }
    } else if(event.type == SDL_MOUSEBUTTONDOWN) {
        cout << "MOUSE CLICKED" << endl;
    }
}

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
            drawPixel(x, y, colour);
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
            drawPixel(x, y, vec3ToPackedInt(c));
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

    drawPixel(0, window.height-1, vec3ToPackedInt(yellowPixel));
    drawPixel(window.width-1, window.height-1, vec3ToPackedInt(greenPixel));

    std::vector<vec3> v;
    v = interpolate(redPixel, yellowPixel, window.height);
    for(int y = 0; y < window.height; y++) {
        drawPixel(0, y, vec3ToPackedInt(v.at(y)));
    }

    v = interpolate(bluePixel, greenPixel, window.height);
    for(int y=0; y < window.height; y++) {
        drawPixel(window.width-1, y, vec3ToPackedInt(v.at(y)));
    }

    for(int y=0; y < window.height; y++) {
        v = interpolate(packedIntToVec3(window.getPixelColour(0,y)),
                                        packedIntToVec3(window.getPixelColour(window.width-1,y)),
                                        window.width);
        for(int x=0; x < window.width; x++) {
            drawPixel(x, y, vec3ToPackedInt(v.at(x)));
        }
    }
}

/////////////////
// 2D TASKS
////////////////

void drawPixel(int x, int y, uint32_t pixel)
{
    if (x < window.width && y < window.height && x > 0 && y > 0) {
        window.setPixelColour(x, y, pixel);
    }    
}

// 2D task 1
void drawLine(CanvasPoint from, CanvasPoint to, Colour colour)
{
    int steps = calcSteps(from,to);
    std::vector<CanvasPoint> points = interpolate(from, to, steps);
    for (int i = 0; i < steps; i++) {
        int x = std::floor(points.at(i).x);
        int y = std::floor(points.at(i).y);
        drawPixel(x, y, colour.toPackedInt());
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
                drawPixel(x, y, colour.toPackedInt());
        }

        // Fill bottom triangle (top-to-bottom, left-to-right)
        for (int y = bottomT.vertices[0].y; y < bottomT.vertices[2].y; ++y) {
            float x1 = getXFromY(y, bottomT.vertices[0], bottomT.vertices[2]);
            float x2 = getXFromY(y, bottomT.vertices[1], bottomT.vertices[2]);
            float startX = std::min(x1,x2);
            float endX = std::max(x1,x2);
            for (int x = startX; x <= endX; x++)
                drawPixel(x, y, colour.toPackedInt());
        }
    
    } 
    else {
        for (int y = t.vertices[0].y; y < t.vertices[2].y; ++y) {
            float x1 = getXFromY(y, t.vertices[0], t.vertices[1]);
            float x2 = getXFromY(y, t.vertices[0], t.vertices[2]);
            float startX = std::min(x1,x2);
            float endX = std::max(x1,x2);
            for (int x = startX; x <= endX; x++)
                drawPixel(x, y, colour.toPackedInt());
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
                    drawPixel(x,y,pixel);
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

/* // 2D task 5
void textureMappingTask()
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
            drawPixel(x, y, pixel);
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
            drawPixel(x, y, pixel);
        }
    }
}

// Saving to PPM
void saveImage()
{
    saveImage(window.width, window.height);
}

void saveImage(int width, int height)
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
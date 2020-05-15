#include <ModelTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <glm/geometric.hpp>
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
void drawModelTriangles(bool fill);
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
void saveImage(string fn);
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
void lookAt(vec3 fromPoint, vec3 toPoint, vec3 vertical);

#define WIDTH 640    
#define HEIGHT 480

string cornellBoxPath = "./cornell-box/";
string cornellBoxMtlPath = cornellBoxPath + "cornell-box.mtl";
string cornellBoxObjPath = cornellBoxPath + "cornell-box.obj";
string hackspaceLogoPath = "./hackspace-logo/";
string hackspaceLogoMtlPath = hackspaceLogoPath + "materials.mtl";
string hackspaceLogoObjPath = hackspaceLogoPath + "logo.obj";
string path; // set in main()

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
vector<ModelTriangle> triangles;
vector<CanvasTriangle> displayTriangles;
vector<vec3> vertices;
vector<Colour> palette;
vector<TexturePoint> texturePoints;

vec3 modelScale = vec3(1,1,1);
float focalLength = window.height;
float cameraSpeed = 2;
float turnSpeedAngle = 45; // degrees
int frameNumber = 0; // keep track of how many frames we have saved
vec3 cameraPosition;
mat3 cameraOrientation;
PixelMap texture;

enum renderModes {
    WIREFRAME,
    RASTER,
    RAYTRACE
};

int renderMode;

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
    
    cout << "[CAM POS] " << to_string(cameraPosition) << endl;
    cout << "[CAM ORIENTATION] " << to_string(cameraOrientation) << endl;
}

// For the sake of Windows
int WinMain(int argc, char* argv[])
{
    main(argc, argv);
    return 0;
}

int main(int argc, char* argv[])
{
    /// CORNELL BOX SETTINGS ↓↓↓
    modelScale = vec3(1, 1, 1);
    path = cornellBoxPath;
    loadMtlFile(cornellBoxMtlPath);
    loadObjFile(cornellBoxObjPath);
    /// CORNELL BOX SETTINGS ↑↑↑
    
    /// LOGO SETTINGS ↓↓↓
    // modelScale = vec3(0.01, 0.01, 0.01);
    // path = hackspaceLogoPath;
    // loadMtlFile(hackspaceLogoMtlPath);
    // loadObjFile(hackspaceLogoObjPath);
    /// LOGO SETTINGS ↑↑↑

    centerCameraPosition();
    renderMode = WIREFRAME;
    cout << "Starting in WIREFRAME mode." << endl;
    // renderMode = RASTER;
    // cout << "Starting in RASTER mode." << endl;
    // renderMode = RAYTRACE;
    // cout << "Starting in RAYTRACE mode." << endl;
    redrawCanvas();
    
    SDL_Event event;
    while (true) {
        // We MUST poll for events - otherwise the window will freeze !
        if (window.pollForInputEvents(&event)) handleEvent(event);
        update();
        // draw();
        // Need to render the frame at the end, or nothing actually gets shown on the screen !
        window.renderFrame();
    }
}

void lookAt(vec3 fromPoint, vec3 toPoint, vec3 vertical)
{
	vec3 forward = normalize(fromPoint - toPoint);
	vec3 right = glm::cross(normalize(vertical), forward);
	vec3 up = glm::cross(forward, right);
	
	cameraOrientation[0][0] = right.x; 
    cameraOrientation[1][0] = right.y; 
    cameraOrientation[2][0] = right.z; 
    cameraOrientation[0][1] = up.x; 
    cameraOrientation[1][1] = up.y; 
    cameraOrientation[2][1] = up.z; 
    cameraOrientation[0][2] = forward.x; 
    cameraOrientation[1][2] = forward.y; 
    cameraOrientation[2][2] = forward.z;
}

void centerCameraPosition()
{
    float minX = triangles.at(0).vertices[0].x;
    float maxX = triangles.at(0).vertices[0].x;
    float minY = triangles.at(0).vertices[0].y;
    float maxY = triangles.at(0).vertices[0].y;
    float minZ = triangles.at(0).vertices[0].z;
    float maxZ = triangles.at(0).vertices[0].z;
    
    for(ModelTriangle triangle : triangles){
        for (int i = 0; i < 3; i++){
            float wx = triangle.vertices[i].x;
            float wy = triangle.vertices[i].y;
            float wz = triangle.vertices[i].z;
            if (wx <= minX) minX = wx;
            if (wx >= maxX) maxX = wx;
            if (wy <= minY) minY = wy;
            if (wy >= maxY) maxY = wy;
            if (wz <= minZ) minZ = wz;
            if (wz >= maxZ) maxZ = wz;
        }
    }
    int cameraStepBack = 5;
    cameraPosition = {(maxX + minX)/2, (maxY + minY)/2, cameraStepBack};
    cameraOrientation = mat3(vec3(1,0,0),vec3(0,1,0),vec3(0,0,1));
}

void drawModelTriangles(bool fill)
{
    window.clearPixels();
    for (ModelTriangle triangle : triangles) {
        CanvasPoint displayVertices[3];
        bool isVisible = true;
        for (int i = 0; i < 3; i++) {
            vec3 cameraToVertex = triangle.vertices[i] - cameraPosition;
            cameraToVertex = cameraToVertex * cameraOrientation;
            
            // perspective projection
            cameraToVertex.x = focalLength * (cameraToVertex.x / cameraToVertex.z);
            cameraToVertex.y = -focalLength * (cameraToVertex.y / cameraToVertex.z);
            if (cameraToVertex.z > cameraPosition.z && cameraToVertex.x < window.width && cameraToVertex.y < window.height) {
                isVisible = false;
            } else {
                // calculate window coordinates and scale
                CanvasPoint c(round((window.width/2) - (cameraToVertex.x)),
                              round((window.height/2) - (cameraToVertex.y)),
                              triangle.texturePoints[i]);
                
                displayVertices[i] = c;
            }
        }
        
        if (isVisible) {
            CanvasTriangle t(displayVertices[0],
                             displayVertices[1], 
                             displayVertices[2],
                             triangle.colour);
            if (fill) {
                if (t.vertices[0].texturePoint.x == -1) {
                    // If there's no texture, just fill the triangle with its colour
                    drawFilledTriangle(t);
                } else {
                    // drawFilledTriangle(t, RED);
                    drawFilledTriangle(t, texture);
                }
            } else { 
                drawStrokedTriangle(t);
            }
        }
    }
}

RayTriangleIntersection getClosestIntersection(int x, int y)
{
    RayTriangleIntersection closestIntersection;
    closestIntersection.distanceFromCamera = INFINITY;

    // A position along a ray can be represented as:
    // [1]      position = startPoint + scalar * direction
    //          r        = s          + t      * d
    
    // Our startPoint is the camera's position, the global variable vec3 cameraPosition
    // Given a triangular plane with vertices p0, p1, p2
    for (ModelTriangle triangle : triangles) {
        // Point r which intersects this triangular plane is defined as: 
        // [2]      r = p0 + u(p1 - p0) + v(p2 - p0)
        // Simplify [2] by referring to the differences between points as edges
        // [3]      r = p0 + ue0 + ve1
        vec3 p0 = triangle.vertices[0];
        vec3 p1 = triangle.vertices[1];
        vec3 p2 = triangle.vertices[2];
        vec3 e0 = p1 - p0;
        vec3 e1 = p2 - p0;
        
        // Calculate the ray's direction using the camera's position and the (x,y) of the pixel the ray intersects
        float xPos = ((window.width  / 2) - x);
        float yPos = (y - (window.height / 2));
        float zPos = focalLength;
        vec3 rayDirection = normalize(cameraPosition - vec3(xPos, yPos, zPos)) * cameraOrientation;
        
        // Combining [3] with [1], we get:
        // [4]      p0 + ue0 + ve1 = s + t * d
        // Rearrange [4] to get:
        // [5]      -t * d + ue0 + ve1 = s - p0
        // Represent [5] in matrix form:
        // [6]      [ -d.x e0.x e1.x ]   [ t ]   [ s.x - p0.x ]
        //          [ -d.y e0.y e1.y ] • [ u ] = [ s.y - p0.y ]
        //          [ -d.z e0.z e1.z ]   [ v ]   [ s.z - p0.z ]
        //          ^ "DEMatrix"                 ^ "SPVector"
        vec3 SPVector = cameraPosition - p0;
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
            vec3 intersectionPoint = triangle.vertices[0] + u*e0 + v*e1;

            // If this intersection point is closer to the camera than the previous closest intersection
            if (closestIntersection.distanceFromCamera > distanceFromCamera) {
                //if ((intersectionPoint.z > cameraPosition.z && intersectionPoint.x < window.width && intersectionPoint.y < window.height)) {
                    // Then the new intersection we have found is the new closest intersection
                    closestIntersection = RayTriangleIntersection(intersectionPoint, distanceFromCamera, triangle);
                //}
            }
        }
    }
    
    // After iterating over every triangle, we will now have found the intersection closest to the camera
    return closestIntersection;
}

void renderWireframeCanvas()
{
    drawModelTriangles(false);
}

void rasteriseCanvas()
{
    drawModelTriangles(true);
}

int applyShading(int v, float brightness)
{
    int x = (int) round(v * brightness);
    if (x > 255) return 255;
    if (x < 0) return 0;
    return x;
}

class LightSource
{
    public:
        vec3 position;
        float intensity;
        
        LightSource()
        {
            position = vec3(0,0,0);
            intensity = 0;
        }
        
        LightSource(vec3 pos)
        {
            position = pos;
            intensity = 0;
        }
        
        LightSource(vec3 pos, float lightIntensity)
        {
            position = pos;
            intensity = lightIntensity;
        }
};

Colour getColourFromIntersection(RayTriangleIntersection intersection, LightSource light)
{
    Colour colour = BLACK;
    vec3 lightBulb = light.position;
    float intensity = light.intensity;
    if (intersection.distanceFromCamera != INFINITY) {
        // Diffuse lighting
        float distanceFromLight = distance(lightBulb, intersection.intersectionPoint);

        // Proximity
        float proximity = 1 / (float)(4 * (float)pi<float>() * distanceFromLight * distanceFromLight);
        
        // Get the triangular plane and its vertices
        ModelTriangle t = intersection.intersectedTriangle;
        vec3 p0 = t.vertices[0];
        vec3 p1 = t.vertices[1];
        vec3 p2 = t.vertices[2];
        
        // Calculate normal
        vec3 e0 = p1 - p0;
        vec3 e1 = p2 - p0;
        vec3 normal = glm::cross(e0, e1);

        // Light a surface receives is directly proportional to the angle of incidence
        vec3 pointToLight = lightBulb - intersection.intersectionPoint;
        float angleOfIncidence = normalize(glm::dot(normal, normalize(pointToLight)));
        
        // Calculate the brightness
        float brightness = intensity * proximity * angleOfIncidence;

        // Ambient lighting
        float lighting_floor = 0.4f;
        brightness = std::max(lighting_floor, brightness);
        
        // Apply the shading
        colour = intersection.intersectedTriangle.colour;
        colour.red = applyShading(colour.red, brightness);
        colour.green = applyShading(colour.green, brightness);
        colour.blue = applyShading(colour.blue, brightness);
    }
    return colour;
}

void raytraceCanvas()
{
    // Iterate over every pixel in the canvas
    for (int y = 0; y < window.height; y++) {
        for (int x = 0; x < window.width; x++) {
            // Shoot a ray from the camera such that it intersects this pixel of the canvas
            RayTriangleIntersection closest = getClosestIntersection(x, y);
            
            // Position our proximity light source at the centre of the ceiling light vertices
            LightSource lightBulb(vec3(((-0.884011 +  0.415989) / 2),
                                       (( 5.219334 +  5.218497) / 2) - 0.4,
                                       ((-2.517968 + -3.567968) / 2)),
                                  200); // intensity

            Colour colour = getColourFromIntersection(closest, lightBulb);            
            drawPixel(x, y, colour.toPackedInt());
        }
    }
}

void update()
{
    // Function for performing animation (shifting artifacts or moving the camera)
}

void changeRenderMode(int mode)
{
    renderMode = mode;
    redrawCanvas();
}

void tiltCamera(float angleDegrees)
{
    float rotationAngle = glm::radians(angleDegrees);
    float cosA = cos(rotationAngle);
    float sinA = sin(rotationAngle);
    mat3 rotateXAxis(vec3(1,0,0),vec3(0,cosA,-sinA),vec3(0,sinA,cosA));
    
    cameraOrientation = cameraOrientation * rotateXAxis;
    redrawCanvas();
    cout << "Tilting camera " << angleDegrees << " degrees." << endl;
}

void panCamera(float angleDegrees)
{
    float rotationAngle = glm::radians(angleDegrees);
    float cosA = cos(rotationAngle);
    float sinA = sin(rotationAngle);
    mat3 rotateYAxis(vec3(cosA,0,sinA),vec3(0,1,0),vec3(-sinA,0,cosA));
    
    cameraOrientation = cameraOrientation * rotateYAxis;
    redrawCanvas();
    cout << "Panning camera " << angleDegrees << " degrees." << endl;
}

void spinCamera(float angleDegrees)
{
    float rotationAngle = glm::radians(angleDegrees);
    float cosA = cos(rotationAngle);
    float sinA = sin(rotationAngle);
    mat3 rotateZAxis(vec3(cosA,-sinA,0),vec3(sinA,cosA,0),vec3(0,0,1));
    
    cameraOrientation = cameraOrientation * rotateZAxis;
    redrawCanvas();
    cout << "Spinning camera " << angleDegrees << " degrees." << endl;
}

void handleEvent(SDL_Event event)
{
	vec3 rightCO = vec3(cameraOrientation[0][0], cameraOrientation[1][0], cameraOrientation[2][0]);
	vec3 upCO = vec3(cameraOrientation[0][1], cameraOrientation[1][1], cameraOrientation[2][1]);
	vec3 forwardCO = vec3(cameraOrientation[0][2], cameraOrientation[1][2], cameraOrientation[2][2]);
	
    if(event.type == SDL_KEYDOWN) {
        if(event.key.keysym.sym == SDLK_LEFT) {
            cameraPosition -= cameraSpeed  * rightCO;
            redrawCanvas();
            cout << "LEFT: Camera shifted left." << endl;
        } else if(event.key.keysym.sym == SDLK_RIGHT) {
            cameraPosition += cameraSpeed  * rightCO;
            redrawCanvas();
            cout << "RIGHT: Camera shifted right." << endl;
        } else if(event.key.keysym.sym == SDLK_UP) {
            cameraPosition += cameraSpeed * upCO;
            redrawCanvas();
            cout << "UP: Camera shifted up." << endl;
        } else if(event.key.keysym.sym == SDLK_DOWN) {
            cameraPosition -= cameraSpeed * upCO;
            redrawCanvas();
            cout << "DOWN: Camera shifted down." << endl;
        } else if(event.key.keysym.sym == SDLK_f) {
            changeRenderMode(WIREFRAME);
            cout << "f KEY: Switched to WIREFRAME MODE." << endl;
        } else if(event.key.keysym.sym == SDLK_r) {
            changeRenderMode(RASTER);
            cout << "r KEY: Switched to RASTER MODE." << endl;
        } else if(event.key.keysym.sym == SDLK_t) {
            changeRenderMode(RAYTRACE);
            cout << "t KEY: Switched to RAYTRACE MODE." << endl;
        } else if(event.key.keysym.sym == SDLK_a) {
            cout << "a KEY: ";
            panCamera(turnSpeedAngle);
        } else if(event.key.keysym.sym == SDLK_d) {
            cout << "d KEY: ";
            panCamera(-turnSpeedAngle);
        } else if(event.key.keysym.sym == SDLK_w) {
            cout << "w KEY: ";
            tiltCamera(turnSpeedAngle);
        } else if(event.key.keysym.sym == SDLK_s) {
            cout << "s KEY: ";
            tiltCamera(-turnSpeedAngle);
        } else if(event.key.keysym.sym == SDLK_q) {
            cout << "q KEY: ";
            spinCamera(turnSpeedAngle);
        } else if(event.key.keysym.sym == SDLK_e) {
            cout << "e KEY: ";
            spinCamera(-turnSpeedAngle);
        } else if(event.key.keysym.sym == SDLK_c) {
            centerCameraPosition();
            redrawCanvas();
            cout << "c KEY: Camera reset." << endl;
        } else if(event.key.keysym.sym == SDLK_p) {
            string filename = "frame_" + to_string(frameNumber) + ".ppm";
            saveImage(filename);
            frameNumber++;
        }
    } else if(event.type == SDL_MOUSEBUTTONDOWN) {
        cout << "MOUSE CLICKED" << endl;
    } else if(event.type == SDL_MOUSEWHEEL) {
        if(event.wheel.y > 0) // scroll up
        {
            cameraPosition -= cameraSpeed * forwardCO;
            redrawCanvas();
            cout << "SCROLL UP: Camera shifted inwards." << endl;
        }
        else if(event.wheel.y < 0) // scroll down
        {
            cameraPosition += cameraSpeed * forwardCO;
            redrawCanvas();
            cout << "SCROLL DOWN: Camera shifted outwards." << endl;
        }
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
    // drawStrokedTriangle(t, t.colour);
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
        //cout << "PPM file '" << fn << "' successfully loaded!" << endl;
    } else {
        cout << "ERROR: Failed to load PPM file!" << endl;
    }

    return img;
}

void drawImage()
{
    PixelMap img = loadPixelMap("texture.ppm");
    drawPixelMap(img);
}

// 2D task 5
void textureMappingTask()
{
    PixelMap img = loadPixelMap("texture.ppm");
    CanvasTriangle t(CanvasPoint(160, 10,TexturePoint(195,  5)),
                     CanvasPoint(300,230,TexturePoint(395,380)),
                     CanvasPoint( 10,150,TexturePoint( 65,330)));
    drawFilledTriangle(t, img);
    drawStrokedTriangle(t);
}

std::vector<TexturePoint> interpolate(TexturePoint from, TexturePoint to, int steps)
{
    std::vector<TexturePoint> v = { from };
    for (int i = 0; i < steps; i++) {
        v.push_back(TexturePoint(interpolationStep(v.back().x, from.x, to.x, steps),
                                 interpolationStep(v.back().y, from.y, to.y, steps)));
    }
    return v;
}

void fillFlatTriangle(CanvasTriangle t, PixelMap img)
{
    CanvasPoint top    = t.vertices[0];
    CanvasPoint middle = t.vertices[1];
    CanvasPoint bottom = t.vertices[2];
    int height = abs(bottom.y - top.y);
    
    std::vector<CanvasPoint> canvasStartPoints;
    std::vector<CanvasPoint> canvasEndPoints;
    std::vector<TexturePoint> textureStartPoints;
    std::vector<TexturePoint> textureEndPoints;
    if (middle.y == bottom.y) {
        // Triangle with flat bottom
        canvasStartPoints  = interpolate(top, middle, height);
        canvasEndPoints    = interpolate(top, bottom, height);
        textureStartPoints = interpolate(top.texturePoint, middle.texturePoint, height);
        textureEndPoints   = interpolate(top.texturePoint, bottom.texturePoint, height);
    } else {
        // Triangle with flat top
        canvasStartPoints  = interpolate(top,    bottom, height);
        canvasEndPoints    = interpolate(middle, bottom, height);
        textureStartPoints = interpolate(top.texturePoint,    bottom.texturePoint, height);
        textureEndPoints   = interpolate(middle.texturePoint, bottom.texturePoint, height);
    }
    
    for (int row = 0; row < height; row++) {
        CanvasPoint startPoint = canvasStartPoints.at(row);
        CanvasPoint endPoint   = canvasEndPoints.at(row);
        int rowWidth = std::round(endPoint.x - startPoint.x);
        std::vector<TexturePoint> textureRow = interpolate(textureStartPoints.at(row),
                                                           textureEndPoints.at(row),
                                                           rowWidth);

        for (int column = 0; column < rowWidth; column++) {
            int x = startPoint.x + column;
            int y = top.y + row;
            TexturePoint texP = textureRow.at(column);
            uint32_t pixel = img.pixels.at(std::round(texP.x) + (img.width * std::round(texP.y)));
            drawPixel(x, y, pixel);
        }
    }
}

void drawFilledTriangle(CanvasTriangle t, PixelMap img)
{
    // Sort vertices by vertical position (top to bottom)
    t.sortVertices();

    CanvasPoint top = t.vertices[0];
    CanvasPoint middle = t.vertices[1];
    CanvasPoint bottom = t.vertices[2];

    // If the bottom of the triangle isn't flat, divide it
    if (middle.y != bottom.y) {
        // Find the missing vertex to divide the triangle into two flat-bottomed triangles
        float divVy = middle.y;
        float divVx = getXFromY(divVy, top, bottom);
        CanvasPoint divV(divVx, divVy);

        // Calculate this new vertex's corresponding texture point
        int stepsTexTopToBottom = calcSteps(top.texturePoint, bottom.texturePoint);
        float proportion = calcSteps(top, divV) / (float)calcSteps(top, bottom);
        int stepsTexTopToDivV = std::floor(stepsTexTopToBottom * proportion);
        std::vector<TexturePoint> texPoints = interpolate(top.texturePoint, bottom.texturePoint, stepsTexTopToBottom);
        divV.texturePoint = texPoints.at(stepsTexTopToDivV);

        // Fill the top triangle
        fillFlatTriangle(CanvasTriangle(top, middle, divV), img);
        
        // Fill the bottom triangle
        t = CanvasTriangle(middle, divV, bottom);
    }

    fillFlatTriangle(t, img);
}

// Saving to PPM
void saveImage(string fn)
{
    PixelMap img;

    img.width = window.width;
    img.height = window.height;
    img.pixels = {};
    
    for (int row = 0; row < img.height; row++) {
        for (int column = 0; column < img.width; column++) {
            img.pixels.push_back(window.getPixelColour(column, row));
        }
    }
    
    ofstream myfile (fn, ios::binary);
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
    
    cout << "SAVED WINDOW AS PPM IMAGE FILE \"" << fn << "\"" << endl;
}

bool loadObjFile(string filepath)
{
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
            // cout << line << endl;
            string *tokens = split(line, delim);
            
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
                vec3 vertex = vec3(v1, v2, v3) * modelScale;
                vertices.push_back(vertex);
            }
			
			// vt line 
			else if (vt.compare(tokens[0]) == 0){
                float v1 = stof(tokens[1]) * (texture.width  - 1);
                float v2 = stof(tokens[2]) * (texture.height - 1);
                TexturePoint tP(v1, v2);
                texturePoints.push_back(tP);
                //cout << "Texture point #" << texturePoints.size() << ": " << tP;
            }
            
            // f line
            else if (f.compare(tokens[0]) == 0) {
                ModelTriangle triangle;
                bool noTexture = true;
                
                // A face consists of three vertices, so let's iterate over them as we read them in
                for (int i = 0; i < 3; i++) {
                    // Split the face values with the delimiter '/'
                    string *faceValues = split(tokens[i + 1], '/');
                    
                    // The 0th token is the index of the face's current vertex
                    int vertexIndex = (stoi(faceValues[0])) - 1;
                    
                    // If the 1th token isn't empty, this vertex also has an associated texture point
                    if (faceValues[1].compare("") != 0) {
                        noTexture = false;
                        int texturePointIndex = (stoi(faceValues[1])) - 1;
                        
                        triangle.colour = WHITE;
                        triangle.texturePoints[i] = texturePoints.at(texturePointIndex);
                    }
                    
                    triangle.vertices[i] = vertices.at(vertexIndex);
                }

                if (noTexture) {
                    // Search the palette for a colour which matches the specified colour name                    
                    int paletteIndex = 0;
                    while (colorName.compare(palette.at(paletteIndex).name) != 0 && paletteIndex < (int) palette.size()) {
                        // Keep looping until we find a match, or reach the end of the palette
                        paletteIndex++;
                    }

                    if (paletteIndex == (int) palette.size()) {
                        // If the palette is empty, or no match was found, but the face doesn't use a texture...
                        triangle.colour = Colour(rand() % 255, rand() % 255, rand() % 255);
                        // ...let the face use a randomly generated colour instead!
                    } else {
                        // Otherwise, set the face's colour to be the colour from the palette which had a matching name
                        triangle.colour = palette.at(paletteIndex);
                    }
                }
                
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
        
        //cout << "I am finished with obj" << endl; 
        
        file.close();
    } else {
        cout << "ERROR: Failed to load OBJ file!" << endl;
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
        string mapkd = "map_Kd";
        string colorName;

        while (getline(file, line)) {
            //cout << line.c_str();
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
            // map_Kd line
            else if (mapkd.compare(tokens[0]) == 0){
                //cout << "map_Kd match" << endl;
                string fileName = tokens[1];
                texture = loadPixelMap(path + fileName);
            } else {
                if (line != ""){
                    cout << "undefined line in MTL" << endl;
                    cout << line << endl;
                }
            }
        }
        
        // cout << "I am finished with mtl" << endl; 
        
        file.close();
    } else {
        cout << "ERROR: Failed to load MTL file!" << endl;
        return false;
    }
    return true;
}
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

// CONSTANTS

#define WIDTH 640    
#define HEIGHT 480


// ===================== FUNCTIONS ========================================

void handleEvent(SDL_Event event);

// LOADING
bool loadMtlFile(string filepath);
bool loadObjFile(string filepath);
PixelMap loadPixelMap(string fn);

// DRAWING
void drawModelTriangles(bool fill);
bool drawPixelMap(PixelMap img, int startX, int startY);
bool drawPixelMap(PixelMap img);
void drawFilledTriangle(CanvasTriangle t, PixelMap img);
void drawFilledTriangle(CanvasTriangle triangle, Colour colour);
void drawFilledTriangle(CanvasTriangle triangle);
void fillFlatTriangle(CanvasTriangle t, PixelMap img);
void drawStrokedTriangle(CanvasTriangle triangle, Colour colour);
void drawStrokedTriangle(CanvasTriangle triangle);
void drawLine(CanvasPoint from, CanvasPoint to, Colour colour);
void drawLine(TexturePoint from, TexturePoint to, Colour colour);
void drawPixel(int x, int y, uint32_t pixel);
void drawImage();


// CANVAS MODE
void renderWireframeCanvas();
void rasteriseCanvas();
void raytraceCanvas();
void redrawCanvas();
void changeRenderMode(int mode);


// INTERPOLATION
float interpolationStep(float start, float from, float to, int steps);
std::vector<float> interpolate(float from, float to, int steps);
std::vector<vec3> interpolate(vec3 from, vec3 to, int steps);
std::vector<CanvasPoint> interpolate(CanvasPoint from, CanvasPoint to, int steps);
int calcSteps(float fromX, float fromY, float toX, float toY);
int calcSteps(CanvasPoint from, CanvasPoint to);
int calcSteps(vec3 from, vec3 to);
float getXFromY(float y, CanvasPoint from, CanvasPoint to);


// SAVE
void saveImage(string fn);


// MATHS
void centerCameraPosition();
float calcPointBrightness(RayTriangleIntersection intersection);
void lookAt(vec3 fromPoint, vec3 toPoint, vec3 vertical);
void transformVertexCoordinatesToCanvas();
RayTriangleIntersection getClosestIntersection(vec3 startPoint, vec3 ray);
Colour shadeColour(Colour colour, float brightness);


// ==================== VARIABLES ======================================

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
vec3 cameraPosition;
mat3 cameraOrientation;
PixelMap texture;

// INITIAL SETTING
vec3 modelScale = vec3(1,1,1);
float focalLength = window.height;
float cameraSpeed = 2;
float turnSpeedAngle = 45; // degrees
int frameNumber = 0; // keep track of how many frames we have saved

// LIGHT
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

LightSource globalLight(vec3(0,0,0),0);


// RENDER

enum renderModes {
    WIREFRAME,
    RASTER,
    RAYTRACE
};
int renderMode;



// WINDOWS
int WinMain(int argc, char* argv[])
{
    main(argc, argv);
    return 0;
}


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
    
    //cout << "[CAM POS] " << to_string(cameraPosition) << endl;
    //cout << "[CAM ORIENTATION] " << to_string(cameraOrientation) << endl;
}


// ======= MAIN ================
int main(int argc, char* argv[])
{
    /// CORNELL BOX SETTINGS ↓↓↓
    //modelScale = vec3(1, 1, 1);
    //path = cornellBoxPath;
    //loadMtlFile(cornellBoxMtlPath);
    //loadObjFile(cornellBoxObjPath);
    //globalLight = LightSource(vec3(-0.234011, 5.2129155,-3.042968), 200); // pos: white light in box
    /// CORNELL BOX SETTINGS ↑↑↑
    
    /// LOGO SETTINGS ↓↓↓
    modelScale = vec3(0.01, 0.01, 0.01);
    path = hackspaceLogoPath;
    loadMtlFile(hackspaceLogoMtlPath);
    loadObjFile(hackspaceLogoObjPath);
    globalLight = LightSource(vec3(2.820000, 2.665000, 3.000000), 120); // pos: to the right in front
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
        // Need to render the frame at the end, or nothing actually gets shown on the screen !
        window.renderFrame();
    }
}



// ============================= RENDER MODE ===============================

void changeRenderMode(int mode)
{
    renderMode = mode;
    redrawCanvas();
}

void renderWireframeCanvas()
{
    drawModelTriangles(false);
}

void rasteriseCanvas()
{
    drawModelTriangles(true);
}

void raytraceCanvas()
{
    window.clearPixels();
    // Cast rays from the camera such that each pixel is intersected
    for (int y = 0; y < window.height; y++) {
        for (int x = 0; x < window.width; x++) {
            // Calculate pixel's world-space pos
            vec3 pixelPosition = vec3((window.width  / 2) - x, y - (window.height / 2), focalLength);
            
            // Cast ray from camera through pixel
            vec3 ray = (cameraPosition - pixelPosition) * cameraOrientation;
            RayTriangleIntersection intersection = getClosestIntersection(cameraPosition, ray);

            if (intersection.distanceFromRayOrigin != std::numeric_limits<float>::infinity()) {
                float brightness = calcPointBrightness(intersection);
                Colour c = shadeColour(intersection.intersectedTriangle.colour, brightness);
                drawPixel(x, y, c.toPackedInt());
            }
        }
    }
}



// ================================ MATHS ======================================

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

int applyShading(int v, float brightness)
{
    v = round(v * brightness);
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return v;
}

Colour shadeColour(Colour colour, float brightness)
{
    colour.red   = applyShading(colour.red,   brightness);
    colour.green = applyShading(colour.green, brightness);
    colour.blue  = applyShading(colour.blue,  brightness);
    return colour;
}

float calcPointBrightness(RayTriangleIntersection intersection)
{
    vec3 point = intersection.intersectionPoint;
    // Diffuse lighting
    float distanceFromLight = distance(globalLight.position, point);

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
    vec3 pointToLight = globalLight.position - point;
    float angleOfIncidence = normalize(glm::dot(normal, normalize(pointToLight)));
    
    // Calculate the brightness
    float brightness = globalLight.intensity * proximity * angleOfIncidence;

    // Ambient lighting
    float lighting_floor = 0.2f;
    brightness = std::max(lighting_floor, brightness);

    return brightness;
}

bool onPlane(float u, float v)
{
    return (0.0f <= u && u <= 1.0f && 0.0f <= v && v <= 1.0f && (u + v) <= 1.0f);
}

RayTriangleIntersection getClosestIntersection(vec3 startPoint, vec3 ray)
{
    RayTriangleIntersection closestIntersection;
    closestIntersection.distanceFromRayOrigin = std::numeric_limits<float>::infinity();

    // Iterate over every triangle in the world to find the point closest from the ray's starting point
    for (ModelTriangle triangle : triangles){
        // position (r) = startPoint + scalar (t) * direction (d)
        // r = p0 + u(p1 - p0) + v(p2 - p0)
        // Therefore p0 + u(p1 - p0) + v(p2 - p0) = s + t * d
        // Rearrange and solve for t, u and v:
        vec3 e0 = triangle.vertices[1] - triangle.vertices[0];
        vec3 e1 = triangle.vertices[2] - triangle.vertices[0];    
        vec3 SPVector = startPoint - triangle.vertices[0];
        mat3 DEMatrix(-normalize(ray), e0, e1);
        vec3 tuv = glm::inverse(DEMatrix) * SPVector;
        
        float t = tuv.x; // distance from the ray's origin
        float u = tuv.y; // triangular plane u co-ord
        float v = tuv.z; // triangular plane z co-ord

        if (onPlane(u, v)) {
            // Transpose (u,v) co-ordinates to world-space
            vec3 intersection = triangle.vertices[0] + (u * e0) + (v * e1);

            // If the intersection point is closer to the camera than the previous closest intersection
            if (closestIntersection.distanceFromRayOrigin > t) {
                // Then update the closest intersection stored
                closestIntersection = RayTriangleIntersection(intersection, t, triangle);
            }
        }
    }
    
    // After iterating over every triangle, we will now have found the intersection closest to the camera
    return closestIntersection;
}


// =================== CAMERA ACTION ========================================

void tiltCamera(float angleDegrees)
{
    float rotationAngle = glm::radians(angleDegrees);
    float cosA = cos(rotationAngle);
    float sinA = sin(rotationAngle);
    mat3 rotateXAxis(vec3(1,0,0),vec3(0,cosA,sinA),vec3(0,-sinA,cosA));
    
    cameraOrientation = cameraOrientation * rotateXAxis;
    redrawCanvas();
    cout << "Tilting camera " << angleDegrees << " degrees." << endl;
}

void panCamera(float angleDegrees)
{
    float rotationAngle = glm::radians(angleDegrees);
    float cosA = cos(rotationAngle);
    float sinA = sin(rotationAngle);
    mat3 rotateYAxis(vec3(cosA,0,-sinA),vec3(0,1,0),vec3(sinA,0,cosA));
    
    cameraOrientation = cameraOrientation * rotateYAxis;
    redrawCanvas();
    cout << "Panning camera " << angleDegrees << " degrees." << endl;
}

void spinCamera(float angleDegrees)
{
    float rotationAngle = glm::radians(angleDegrees);
    float cosA = cos(rotationAngle);
    float sinA = sin(rotationAngle);
    mat3 rotateZAxis(vec3(cosA,sinA,0),vec3(-sinA,cosA,0),vec3(0,0,1));
    
    cameraOrientation = cameraOrientation * rotateZAxis;
    redrawCanvas();
    cout << "Spinning camera " << angleDegrees << " degrees." << endl;
}


// ========================= EVENT HANDLER ======================================

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
            panCamera(-turnSpeedAngle);
        } else if(event.key.keysym.sym == SDLK_d) {
            cout << "d KEY: ";
            panCamera(turnSpeedAngle);
        } else if(event.key.keysym.sym == SDLK_w) {
            cout << "w KEY: ";
            tiltCamera(-turnSpeedAngle);
        } else if(event.key.keysym.sym == SDLK_s) {
            cout << "s KEY: ";
            tiltCamera(turnSpeedAngle);
        } else if(event.key.keysym.sym == SDLK_q) {
            cout << "q KEY: ";
            spinCamera(-turnSpeedAngle);
        } else if(event.key.keysym.sym == SDLK_e) {
            cout << "e KEY: ";
            spinCamera(turnSpeedAngle);
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
            cout << "SCROLL UP: Camera shifted forward." << endl;
        }
        else if(event.wheel.y < 0) // scroll down
        {
            cameraPosition += cameraSpeed * forwardCO;
            redrawCanvas();
            cout << "SCROLL DOWN: Camera shifted backwards." << endl;
        }
    }
}



// =========== INTERPOLATION FUNCTIONS ============================


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


std::vector<TexturePoint> interpolate(TexturePoint from, TexturePoint to, int steps)
{
    std::vector<TexturePoint> v = { from };
    for (int i = 0; i < steps; i++) {
        v.push_back(TexturePoint(interpolationStep(v.back().x, from.x, to.x, steps),
                                 interpolationStep(v.back().y, from.y, to.y, steps)));
    }
    return v;
}


// ============ DRAWING FUNCTIONS ====================================
 

void drawPixel(int x, int y, uint32_t pixel)
{
    if (x < window.width && y < window.height && x > 0 && y > 0) {
        window.setPixelColour(x, y, pixel);
    }    
}


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


void drawFilledTriangle(CanvasTriangle t)
{
    drawFilledTriangle(t, t.colour);
}


void drawModelTriangles(bool fill)
{
    window.clearPixels();
    for (ModelTriangle triangle : triangles) {
        CanvasPoint displayVertices[3];
        for (int i = 0; i < 3; i++) {
            vec3 cameraToVertex = triangle.vertices[i] - cameraPosition;
            cameraToVertex = cameraOrientation * cameraToVertex;
            
            // perspective projection
            cameraToVertex.x = focalLength * (cameraToVertex.x / cameraToVertex.z);
            cameraToVertex.y = focalLength * (cameraToVertex.y / cameraToVertex.z);

            // calculate window coordinates and scale
            //int xPos = round((window.width/2) - (cameraToVertex.x));
            //int yPos = round((window.height/2) - (cameraToVertex.y));
            int xPos = -cameraToVertex.x + (window.width / 2);
            int yPos = cameraToVertex.y + (window.height / 2);
            CanvasPoint c(xPos, yPos, triangle.texturePoints[i]);
            
            displayVertices[i] = c;
        }
        
        CanvasTriangle t(displayVertices[0],
                         displayVertices[1], 
                         displayVertices[2],
                         triangle.colour);
        if (fill) {
            if (t.vertices[0].texturePoint.x == -1) {
                // If there's no texture, just fill the triangle with its colour
                drawFilledTriangle(t);
            } else {
                drawFilledTriangle(t, RED);
                drawFilledTriangle(t, texture);
            }
        } else { 
            drawStrokedTriangle(t);
        }
    }
}


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


void drawImage()
{
    PixelMap img = loadPixelMap("texture.ppm");
    drawPixelMap(img);
}



// ================== TEXTURE MAPPING ==============================


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


// ======================== SAVING TO PPM FILE================================
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
    
    cout << "Saved window as .ppt file \"" << fn << "\"" << endl;
}


// =================== LOADING FUNCTIONS =====================================


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
        
        //cout << "Loading .OBJ finished." << endl; 
        
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
            string *tokens = split(line, delim);
            
            // newmtl line
            if (newmtl.compare(tokens[0]) == 0){
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
        
        //cout << "Loading .MTL finished." << endl; 
        
        file.close();
    } else {
        cout << "ERROR: Failed to load MTL file!" << endl;
        return false;
    }
    return true;
}





/* ======================== ANTI-ALIASING CHANGES ===========================


// replace drawLine with drawAALine

void drawStrokedTriangle(CanvasTriangle triangle, Colour colour)
{
	
    drawAALine(triangle.vertices[0], triangle.vertices[1], colour);
	drawAALine(triangle.vertices[0], triangle.vertices[2], colour);
    drawAALine(triangle.vertices[1], triangle.vertices[2], colour);
}


// ------------ Some math functions ----
  
//returns integer part of a floating point number 
int iPartOfNumber(float x) 
{ 
    return (int)x; 
} 
  
//rounds off a number 
int roundNumber(float x) 
{ 
    return iPartOfNumber(x + 0.5) ; 
} 
  
//returns fractional part of a number 
float fPartOfNumber(float x) 
{ 
    if (x>0) return x - iPartOfNumber(x); 
    else return x - (iPartOfNumber(x)+1); 
  
} 
  
//returns 1 - fractional part of number 
float rfPartOfNumber(float x) 
{ 
    return 1 - fPartOfNumber(x); 
} 


// min and max of 3 floats 

float max(float a, float b, float c) {
   return ((a > b)? (a > c ? a : c) : (b > c ? b : c));
}
float min(float a, float b, float c) {
   return ((a < b)? (a < c ? a : c) : (b < c ? b : c));
}


// new drawLine below
// the following code goes through the correct pixels of a line
// but then still draws it with full brightness 
// which has the BOLD LINE effect

void drawAALine(CanvasPoint from, CanvasPoint to, Colour colour) 
{ 
	
	
	// get RGB color to HSL
	// get the L value and reduce it by brightness percentage
	// recalculate S and L values of HSL
	// construct new RGB from updated HSL
	
	
	
	
	//float r = colour.red / 255;
	//float g = colour.green / 255;
	//float b = colour.blue / 255;
	//float minrgb = min(r, g, b);
	//float maxrgb = max(r, g, b);
	//float lumi = (minrgb + maxrgb)/2;


	int x0 = from.x;
	int y0 = from.y;
	int x1 = to.x; 
	int y1 = to.y;
	
    bool steep = abs(y1 - y0) > abs(x1 - x0); 
	
    if (steep) 
    { 
	// swap values
		int tmp = x0;
		x0 = y0;
		y0 = tmp;
		
		tmp = x1;
		x1 = y1;
		y1 = tmp;
    } 
	
    if (x0 > x1) 
    { 	
		// swap values
		int tmp = x0;
		x0 = x1;
		x1 = tmp;
		tmp = y0;
		y0 = y1;
		y1 = tmp;
    } 
  

    float dx = x1-x0; 
    float dy = y1-y0; 
    float gradient = dy/dx; 
    if (dx == 0.0) 
        gradient = 1; 
  
    int xpxl1 = x0; 
    int xpxl2 = x1; 
    float intersectY = y0; 
	//float brightness;
  
    if (steep) 
    { 
        int x; 
        for (x = xpxl1 ; x <=xpxl2 ; x++) 
        { 	
            
			CanvasPoint drawp1 = CanvasPoint(iPartOfNumber(intersectY), x);
			//brightness = fPartOfNumber(intersectY);	
			//cHSV.z *= brightness;
			
			
			
			
            drawPixel(drawp1.x, drawp1.y, colour.toPackedInt()); 
			
			CanvasPoint drawp2 = CanvasPoint(iPartOfNumber(intersectY)-1, x);
			//brightness = fPartOfNumber(intersectY);
			//cHSV.z *= brightness;
			
			
            drawPixel(drawp2.x, drawp2.y, colour.toPackedInt()); 
			
			
            intersectY += gradient; 
        } 
    } 
    else
    { 
        int x; 
        for (x = xpxl1 ; x <=xpxl2 ; x++) 
        { 
			CanvasPoint drawp1 = CanvasPoint(x, iPartOfNumber(intersectY));
			//brightness = rfPartOfNumber(intersectY);
			//cHSL.z *= brightness;
			
			
            drawPixel(drawp1.x, drawp1.y, colour.toPackedInt());
			
			
			CanvasPoint drawp2 = CanvasPoint(x, iPartOfNumber(intersectY)-1);
			//brightness = fPartOfNumber(intersectY);
			//cHSL.z *= brightness;			
			
			
            drawPixel(drawp2.x, drawp2.y, colour.toPackedInt());
						  
            intersectY += gradient; 
        } 
    } 
	
} 

// -------------------------------------------------------------------------------------------

*/

















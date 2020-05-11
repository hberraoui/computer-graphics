#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <PixelMap.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>

using namespace std;
using namespace glm;

#define WIDTH 430
#define HEIGHT 395
#define WHITE Colour(255,255,255)
#define RED Colour(255,0,0)
#define GREEN Colour(0,255,0)
#define BLUE Colour(0,0,255)
#define BLACK Colour(0,0,0)

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);

void draw();
void update();
void handleEvent(SDL_Event event);
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

/////////////////
// STRUCTURE
////////////////

int main(int argc, char* argv[])
{
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

// For the sake of Windows
int WinMain(int argc, char* argv[])
{
    main(argc, argv);
    return 0;
}

void update()
{
    // Function for performing animation (shifting artifacts or moving the camera)
}

void draw()
{
    // For animated red noise:
    // drawRedNoise();
}

void handleEvent(SDL_Event event)
{
    if(event.type == SDL_KEYDOWN) {
        if(event.key.keysym.sym == SDLK_LEFT) cout << "LEFT" << endl;
        else if(event.key.keysym.sym == SDLK_RIGHT) cout << "RIGHT" << endl;
        else if(event.key.keysym.sym == SDLK_UP) cout << "UP" << endl;
        else if(event.key.keysym.sym == SDLK_DOWN) cout << "DOWN" << endl;
        else if(event.key.keysym.sym == SDLK_c) window.clearPixels(); // "c" key clears screen
        else if(event.key.keysym.sym == SDLK_r) drawRedNoise(); // "r" key draws random red noise
        else if(event.key.keysym.sym == SDLK_u) drawRandomStrokedTriangle(); // "u" key draws a random unfilled triangle
        else if(event.key.keysym.sym == SDLK_f) drawRandomFilledTriangle(); // "f" key draws a random filled triangle
        else if(event.key.keysym.sym == SDLK_i) drawImage(); // "i" key loads the PPM image and draws it
        else if(event.key.keysym.sym == SDLK_t) textureMappingTask(); // "t" key draws a texture mapped triangle as in 2D Task 5
    }
    else if(event.type == SDL_MOUSEBUTTONDOWN) cout << "MOUSE CLICKED" << endl;
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

    // Test the fill is accurate by drawing the original triangle
    drawStrokedTriangle(t, WHITE);
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
void textureMappingTask()
{
    PixelMap img = loadPixelMap("texture.ppm");
    CanvasTriangle t(CanvasPoint(160, 10,TexturePoint(195,    5)),
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

void drawFilledTriangle(CanvasTriangle t, PixelMap img)
{
    // TODO: Properly implement Task 5: Texture Mapping
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

    // Calculate steps from 0 to 1
    int numberOfStepsFromZeroToOne = topT.vertices[1].y - topT.vertices[0].y; //calcSteps(topT.vertices[0], topT.vertices[1]);
    // cout << "Canvas space 0 to 1: " << numberOfStepsFromZeroToOne << endl;

    // Calculate steps from 0 to 2
    int numberOfStepsFromZeroToTwo = calcSteps(topT.vertices[0], topT.vertices[2]);
    // cout << "Canvas space 0 to 2: " << numberOfStepsFromZeroToTwo << endl;

    // Interpolate from 0 to 1 in the canvas space
    std::vector<CanvasPoint> pixelsTopToLeft = interpolate(topT.vertices[0],
                                                           topT.vertices[1],
                                                           numberOfStepsFromZeroToOne);

    // Interpolate from 0 to 2 in the canvas space
    std::vector<CanvasPoint> pixelsTopToRight = interpolate(topT.vertices[0],
                                                            topT.vertices[2],
                                                            numberOfStepsFromZeroToTwo);

    // Select biggest number of steps
    int numberOfRows = topT.vertices[1].y - topT.vertices[0].y; //std::max(numberOfStepsFromZeroToOne, numberOfStepsFromZeroToTwo);
    // cout << "Selected number of steps: " << numberOfRows << endl; 

    // Interpolate from 0 to 1 in the texture space
    std::vector<TexturePoint> texelsTopToLeft = interpolate(topT.vertices[0].texturePoint,
                                                            topT.vertices[1].texturePoint,
                                                            numberOfRows);
    
    // Interpolate from 0 to 2 in the texture space
    std::vector<TexturePoint> texelsTopToRight = interpolate(topT.vertices[0].texturePoint,
                                                             topT.vertices[2].texturePoint,
                                                             numberOfRows);

    std::vector<TexturePoint> textureRow = {};
    for (int step = 0; step < numberOfRows; step++) {
        CanvasPoint startPoint = pixelsTopToLeft.at(step);
        CanvasPoint endPoint = pixelsTopToRight.at(step);

        int numberOfStepsFromStartToEnd = std::round(endPoint.x - startPoint.x);
        textureRow = interpolate(texelsTopToLeft.at(step),
                                 texelsTopToRight.at(step),
                                 numberOfStepsFromStartToEnd);


        for (int i = 0; i < (int) textureRow.size(); i++) {
            int x = startPoint.x + i;
            int y = topT.vertices[0].y + step;
            TexturePoint texP = textureRow.at(i);
            uint32_t pixel = img.pixels.at(std::round(texP.x) + (img.width * std::round(texP.y)));
            window.setPixelColour(x, y, pixel);
        }
    }
        
    cout << (int) textureRow.size() << endl;
}
#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>

using namespace std;
using namespace glm;

#define WIDTH 320
#define HEIGHT 240
#define WHITE Colour(255,255,255)
#define RED Colour(255,0,0)
#define GREEN Colour(0,255,0)
#define BLUE Colour(0,0,255)
#define BLACK Colour(0,0,0)

bool drawnOnce = false;

void draw();
void update();
void handleEvent(SDL_Event event);
float interpolationStep(float start, float to, float from, int steps);
std::vector<float> interpolate(float from, float to, int steps);
std::vector<vec3> interpolate(vec3 from, vec3 to, int steps);
std::vector<vec3> interpolate(CanvasPoint from, CanvasPoint to, int steps);
int calcSteps(vec3 from, vec3 to);
int calcSteps(CanvasPoint from, CanvasPoint to);
float findXatYOnLine(float y, CanvasPoint from, CanvasPoint to);
void drawRandomFilledTriangle();
void drawFilledTriangle(CanvasTriangle triangle, Colour colour);
void drawFilledTriangle(CanvasTriangle triangle);
void drawFilledFlatBottomTriangle(CanvasTriangle t, Colour c);
void drawRandomStrokedTriangle();
void drawStrokedTriangle(CanvasTriangle triangle, Colour colour);
void drawStrokedTriangle(CanvasTriangle triangle);
void drawLineWrapper();
void drawLine(CanvasPoint from, CanvasPoint to, Colour colour);
uint32_t vec3ToPackedInt(vec3 pixel);
vec3 packedIntToVec3(uint32_t colour);
void rainbowInterpolation();
void greyscaleInterpolation();

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);

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

void draw()
{
  if (!drawnOnce) {
    //greyscaleInterpolation();
    //rainbowInterpolation();
    //drawLineWrapper();
    //drawRandomStrokedTriangle();
    CanvasTriangle t = CanvasTriangle(CanvasPoint(227,41),CanvasPoint(260,94),CanvasPoint(44,209),RED);
    drawFilledTriangle(t);

    drawnOnce = true;
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
    else if(event.key.keysym.sym == SDLK_u) drawRandomStrokedTriangle();
    else if(event.key.keysym.sym == SDLK_f) drawRandomFilledTriangle();
  }
  else if(event.type == SDL_MOUSEBUTTONDOWN) cout << "MOUSE CLICKED" << endl;
}

float interpolationStep(float start, float to, float from, int steps)
{
  return start + (to - from) / steps;
}

std::vector<float> interpolate(float from, float to, int steps)
{
  std::vector<float> v = { from };
    
  for (int i = 0; i < steps; ++i) {
    v.push_back(interpolationStep(v.back(),to,from,steps));
  }
  
  return v;
}

std::vector<vec3> interpolate(vec3 from, vec3 to, int steps)
{
  std::vector<vec3> v = { from };

  for (int i = 0; i < steps; ++i) {
    vec3 newvec(interpolationStep(v.back().x,to.x,from.x,steps),
                interpolationStep(v.back().y,to.y,from.y,steps),
                interpolationStep(v.back().z,to.z,from.z,steps));
    v.push_back(newvec);
  }

  return v;
}

std::vector<vec3> interpolate(CanvasPoint from, CanvasPoint to, int steps)
{
  vec3 fromVec(from.x,from.y,0);
  vec3 toVec(to.x,to.y,0);

  return interpolate(fromVec,toVec,steps);
}

int calcSteps(vec3 from, vec3 to)
{    
  float xDiff = to.x - from.x;
  float yDiff = to.y - from.y;
  return round(std::max(abs(xDiff), abs(yDiff)));
}

int calcSteps(CanvasPoint from, CanvasPoint to)
{    
  float xDiff = to.x - from.x;
  float yDiff = to.y - from.y;
  return round(std::max(abs(xDiff), abs(yDiff)));
}

float findXatYOnLine(float yTarget, CanvasPoint from, CanvasPoint to)
{
  float x = 0;
  
  int steps = calcSteps(from,to);  
  std::vector<vec3> xys = interpolate(from, to, steps);
  for (int i=0; i<steps; i++) {
      if (round(xys.at(i).y) == round(yTarget)) {
          x = xys.at(i).x;
          break;
      }
  }  

  return x;
}

void drawRandomFilledTriangle()
{
  CanvasPoint v0 = CanvasPoint(rand() % window.width, rand() % window.height);
  CanvasPoint v1 = CanvasPoint(rand() % window.width, rand() % window.height);
  CanvasPoint v2 = CanvasPoint(rand() % window.width, rand() % window.height);
  Colour colour = Colour(rand() % 255, rand() % 255, rand() % 255);
  CanvasTriangle triangle = CanvasTriangle(v0, v1, v2, colour);
  
  drawFilledTriangle(triangle);
}

CanvasPoint deriveNewVertex(CanvasTriangle t)
{
    float y = t.vertices[1].y;
    float x = findXatYOnLine(y, t.vertices[0], t.vertices[2]);
    return CanvasPoint(x,y);
}

void drawFilledTriangle(CanvasTriangle t, Colour colour)
{
  // Function for a filled triangle
  // Sort vertices by vertical position (top to bottom)
  t.sortVertices();
  
  // Divide triangle into 2 "flat-bottomed" triangles
  CanvasPoint v = deriveNewVertex(t);
  CanvasTriangle topT = CanvasTriangle(t.vertices[0], t.vertices[1], v);
  CanvasTriangle bottomT = CanvasTriangle(t.vertices[1], v, t.vertices[2]);
  drawStrokedTriangle(topT, colour);
  drawStrokedTriangle(bottomT, colour);

  // Fill top triangle (top-to-bottom, left-to-right)
  
  // Fill bottom triangle (top-to-bottom, left-to-right)
  
  drawStrokedTriangle(t, WHITE);
}

void drawFilledTriangle(CanvasTriangle t)
{
  drawFilledTriangle(t, t.colour);
}

void drawFilledFlatBottomTriangle(CanvasTriangle t, Colour c)
{
  int top = std::min(t.vertices[0].y,t.vertices[2].y);
  int bottom = std::max(t.vertices[0].y,t.vertices[2].y);
  for (int y = top; y <= bottom; ++y) {
    float startX = findXatYOnLine(y, t.vertices[0], t.vertices[1]);
    float endX = findXatYOnLine(y, t.vertices[0], t.vertices[2]);
    drawLine(CanvasPoint(startX, y), CanvasPoint(endX, y), c);
  }
  drawLine(t.vertices[1], t.vertices[2], c);
}

void drawRandomStrokedTriangle()
{
  CanvasPoint v0 = CanvasPoint(rand() % window.width, rand() % window.height);
  CanvasPoint v1 = CanvasPoint(rand() % window.width, rand() % window.height);
  CanvasPoint v2 = CanvasPoint(rand() % window.width, rand() % window.height);
  Colour colour = Colour(rand() % 255, rand() % 255, rand() % 255);
  CanvasTriangle triangle = CanvasTriangle(v0, v1, v2, colour);
  
  drawStrokedTriangle(triangle);
}

void drawStrokedTriangle(CanvasTriangle triangle, Colour colour)
{
  // Function for a stroked triangle
  drawLine(triangle.vertices[0], triangle.vertices[1], colour);
  drawLine(triangle.vertices[0], triangle.vertices[2], colour);
  drawLine(triangle.vertices[1], triangle.vertices[2], colour);
}

void drawStrokedTriangle(CanvasTriangle triangle)
{
  drawStrokedTriangle(triangle, triangle.colour);
}

void drawLineWrapper()
{
  CanvasPoint from = CanvasPoint(10, 20);
  CanvasPoint to = CanvasPoint(50, 50);
  Colour colour = Colour(255,255,255);

  drawLine(from, to, colour);
}

void drawLine(CanvasPoint from, CanvasPoint to, Colour colour)
{
  // Function for drawing lines
  int steps = calcSteps(from,to);  
  std::vector<vec3> xys = interpolate(from, to, steps);
  for (int i=0; i<steps; i++) {
    window.setPixelColour(round(xys.at(i).x), round(xys.at(i).y), colour.toPackedInt());
  }  
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

void rainbowInterpolation()
{
  // Function for drawing two dimension colour interpolation  
  window.clearPixels();
  vec3 redPixel(255,0,0);
  vec3 bluePixel(0,0,255);
  vec3 yellowPixel(255,255,0);
  vec3 greenPixel(0,255,0);
  window.setPixelColour(0, window.height-1, vec3ToPackedInt(yellowPixel));
  window.setPixelColour(window.width-1, window.height-1, vec3ToPackedInt(greenPixel));
  
  std::vector<vec3> v;
  v = interpolate(redPixel, yellowPixel, window.height);
  for(int y=0; y<window.height; y++) {
    window.setPixelColour(0, y, vec3ToPackedInt(v.at(y)));
  }
  
  v = interpolate(bluePixel, greenPixel, window.height);
  for(int y=0; y<window.height; y++) {
    window.setPixelColour(window.width-1, y, vec3ToPackedInt(v.at(y)));
  }

  for(int y=0; y<window.height; y++) {
    v = interpolate(packedIntToVec3(window.getPixelColour(0,y)),
                    packedIntToVec3(window.getPixelColour(window.width-1,y)),
                    window.width);
    for(int x=0; x<window.width; x++) {
      window.setPixelColour(x, y, vec3ToPackedInt(v.at(x)));
    }
  }
}

void greyscaleInterpolation()
{
  // Function for drawing one dimension greyscale interpolation  
  window.clearPixels();
  std::vector<float> v;
  v = interpolate(0, 255, window.width);
  for(int y=0; y<window.height; y++) {
    for(int x=0; x<window.width; x++) {
      vec3 c(v.at(x), v.at(x), v.at(x));
      window.setPixelColour(x, y, vec3ToPackedInt(c));
    }
  }
}

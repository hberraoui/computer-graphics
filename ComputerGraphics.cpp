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

bool drawnOnce = false;

void draw();
void update();
void handleEvent(SDL_Event event);

void greyscaleInterpolation();
void rainbowInterpolation();
void drawLineWrapper();
void drawStrokedTriangleWrapper();

float interpolationStep(float start, float to, float from, int steps);
std::vector<float> interpolate(float from, float to, int steps);
std::vector<vec3> interpolate(vec3 from, vec3 to, int steps);
void drawLine(CanvasPoint from, CanvasPoint to, Colour colour);
void drawStrokedTriangle(CanvasTriangle triangle);

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

uint32_t vec3ToPackedInt(vec3 pixel)
{
  return (255<<24) + (int(pixel.x)<<16) + (int(pixel.y)<<8) + int(pixel.z);
}

vec3 packedIntToVec3(uint32_t colour)
{
  vec3 result( (colour<<8)>>24, (colour<<16)>>24, (colour<<24)>>24 );
  return result;
}

void draw()
{
  if (!drawnOnce) {
    //greyscaleInterpolation();
    //rainbowInterpolation();
    //drawLineWrapper();
    drawStrokedTriangleWrapper();

    drawnOnce = true;
  }
}

void drawStrokedTriangleWrapper()
{
  CanvasPoint v0 = CanvasPoint(10, 20);
  CanvasPoint v1 = CanvasPoint(50, 50);
  CanvasPoint v2 = CanvasPoint(30, 50);
  Colour colour = Colour(255,0,0);
  CanvasTriangle triangle = CanvasTriangle(v0, v1, v2, colour);
  
  drawStrokedTriangle(triangle);
}

void drawStrokedTriangle(CanvasTriangle triangle)
{
  // Function for a stroked triangle
  cout << "[DRAW STROKED TRIANGLE]:" << endl << triangle;
  
  drawLine(triangle.vertices[0], triangle.vertices[1], triangle.colour);
  drawLine(triangle.vertices[0], triangle.vertices[2], triangle.colour);
  drawLine(triangle.vertices[1], triangle.vertices[2], triangle.colour);
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
  cout << "[DRAW LINE]: from " << from << "[DRAW LINE]: to " << to << "[DRAW LINE]: colour " << colour << endl;
  
  vec3 fromVec(from.x,from.y,colour.toPackedInt());
  vec3 toVec(to.x,to.y,colour.toPackedInt());
  
  float xDiff = to.x - from.x;
  float yDiff = to.y - from.y;
  float numberOfSteps = std::max(abs(xDiff), abs(yDiff));
  
  std::vector<vec3> xys;
  xys = interpolate(fromVec, toVec, numberOfSteps);

  for (float i=0.0; i<numberOfSteps; i++) {
    window.setPixelColour(round(xys.at(i).x), round(xys.at(i).y), colour.toPackedInt());
  }  
}

void rainbowInterpolation()
{
  // Function for drawing two dimension colour interpolation
  cout << "[DRAW 2D COLOUR INTERPOLATION RAINBOW]" << endl;
  
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
  cout << "[DRAW 1D GREYSCALE INTERPOLATION GRADIENT]" << endl;
  
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
  else if(event.type == SDL_MOUSEBUTTONDOWN) cout << "MOUSE CLICKED" << endl;
}

// For the sake of Windows
int WinMain(int argc, char* argv[])
{
  main(argc, argv);
  return 0;
}

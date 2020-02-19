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

void draw();
void update();
void handleEvent(SDL_Event event);
bool drawnOnce = 0;

void greyscaleInterpolation();
void rainbowInterpolation();
float interpolationStep(float start, float to, float from, int steps);
std::vector<float> interpolate(float from, float to, int numberOfValues);
std::vector<vec3> interpolate(vec3 from, vec3 to, int numberOfValues);

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

std::vector<float> interpolate(float from, float to, int numberOfValues)
{
  std::vector<float> v = { from };
    
  int steps = numberOfValues - 1;
  for (int i = 0; i < steps; ++i) {
    v.push_back(interpolationStep(v.back(),to,from,steps));
  }
  
  return v;
}

std::vector<vec3> interpolate(vec3 from, vec3 to, int numberOfValues)
{
  std::vector<vec3> v = { from };
    
  int steps = numberOfValues - 1;
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

vec3 colourToVec3(uint32_t colour)
{
  vec3 result( (colour<<8)>>24, (colour<<16)>>24, (colour<<24)>>24 );
  return result;
}
void drawLine(CanvasPoint from, CanvasPoint to, Colour colour);
void draw()
{
  if (!drawnOnce) {
    // greyscaleInterpolation();
    // rainbowInterpolation();
    Colour colour;
    colour.red = 255;
    colour.blue = 255;
    colour.green = 255;
    CanvasPoint from;
    from.x = 30;
    from.y = 32;
    CanvasPoint to;
    to.x = 10;
    to.y = 33;
    drawLine(from, to, colour);
    drawnOnce = true;
  }
}

void drawLine(CanvasPoint from, CanvasPoint to, Colour colour)
{
  // Function for drawing lines
  cout << "Drawing line from (" << from.x << "," << from.y << ") to (" << to.x << "," << to.y << ")" << endl;
  float xDiff = to.x - from.x;
  float yDiff = to.y - from.y;
  float numberOfSteps = std::max(abs(xDiff), abs(yDiff));
  float xStepSize = xDiff / numberOfSteps;
  float yStepSize = yDiff / numberOfSteps;
  for (float i=0.0; i<numberOfSteps; i++) {
    float x = from.x + (xStepSize*i);
    float y = from.y + (yStepSize*i);
    window.setPixelColour(round(x), round(y), colour.toPackedInt());
  }  
}

void greyscaleInterpolation()
{
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

void rainbowInterpolation()
{
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
    v = interpolate(colourToVec3(window.getPixelColour(0,y)),
                    colourToVec3(window.getPixelColour(window.width-1,y)),
                    window.width);
    for(int x=0; x<window.width; x++) {
      window.setPixelColour(x, y, vec3ToPackedInt(v.at(x)));
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

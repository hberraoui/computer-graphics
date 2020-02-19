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

uint32_t vec3ToColour(vec3 pixel)
{
  return (255<<24) + (int(pixel.x)<<16) + (int(pixel.y)<<8) + int(pixel.z);
}

vec3 colourToVec3(uint32_t colour)
{
  vec3 result( (colour<<8)>>24, (colour<<16)>>24, (colour<<24)>>24 );
  return result;
}
void drawLine(float x, float y, Colour colour);
void draw()
{
  // greyscaleInterpolation();
  // rainbowInterpolation();
  Colour colour;
  colour.red = 255;
  colour.blue = 255;
  colour.green = 255;
  drawLine(20,32,colour);
}

void drawLine(float x, float y, Colour colour)
{
  // Function for drawing lines
  cout << "Drawing line from " << x << " to " << y;
}

void greyscaleInterpolation()
{
  window.clearPixels();
  std::vector<float> v;
  v = interpolate(0, 255, window.width);
  for(int y=0; y<window.height; y++) {
    for(int x=0; x<window.width; x++) {
      vec3 c(v.at(x), v.at(x), v.at(x));
      window.setPixelColour(x, y, vec3ToColour(c));
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
  window.setPixelColour(0, window.height-1, vec3ToColour(yellowPixel));
  window.setPixelColour(window.width-1, window.height-1, vec3ToColour(greenPixel));
  
  std::vector<vec3> v;
  v = interpolate(redPixel, yellowPixel, window.height);
  for(int y=0; y<window.height; y++) {
    window.setPixelColour(0, y, vec3ToColour(v.at(y)));
  }
  
  v = interpolate(bluePixel, greenPixel, window.height);
  for(int y=0; y<window.height; y++) {
    window.setPixelColour(window.width-1, y, vec3ToColour(v.at(y)));
  }

  for(int y=0; y<window.height; y++) {
    v = interpolate(colourToVec3(window.getPixelColour(0,y)),
                    colourToVec3(window.getPixelColour(window.width-1,y)),
                    window.width);
    for(int x=0; x<window.width; x++) {
      window.setPixelColour(x, y, vec3ToColour(v.at(x)));
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

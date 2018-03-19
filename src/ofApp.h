#pragma once

#include "ofMain.h"
#include "ofxFlowTools.h"
#include "ofxGui.h"
#include "ofxPostGlitch.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

using namespace flowTools;

class ofApp : public ofBaseApp{

public:
  void setup();
  void update();
  void draw();
  
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  void updateFluid();

  void mainDraw();
  void drawGui();

  void drawFluid();
  void drawStar();
  void drawWave();
  void drawSun();
  void drawConstellation();
  void drawFirefly();
  void drawTerrain();

  ofxIntSlider mode;
  ofxIntSlider alpha;
  ofxIntSlider xCam;
  ofxIntSlider yCam;
  ofxIntSlider zCam;
  ofxIntSlider interval;
  ofxIntSlider toggleTrajectoryDraw;
  ofxIntSlider toggleGlitchDraw;
  ofxIntSlider toggleRotate180;

  ofxPanel gui;
  ofxPanel gui2;
  ofxPanel gui3;

  ofFbo myFbo;
  ofxPostGlitch myGlitch;
  ofMesh mesh;
  ofEasyCam cam;

  void keyPressed(int key);
  void keyReleased(int key);

  ftFluidSimulation fluidSimulation;
  ftDrawMouseForces mouseForces;
  float deltaTime;
  float lastTime;

private:
  ros::NodeHandle n;
  ros::Subscriber scan_sub;
  int drawStar_allow;
  double level_center;
  double level_left;
  double scanValues[726];
  double angle_diff;
  double center;
  double left;
  double depth;
};

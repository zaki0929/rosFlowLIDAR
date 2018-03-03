#pragma once

#include "ofMain.h"
#include "ofxFlowTools.h"
#include "ofxGui.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

using namespace flowTools;

class ofApp : public ofBaseApp{

public:
  void setup();
  void update();
  void draw();
  
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void drawStar();
  void drawWave();
  void drawFluid();

  ofxIntSlider mode;
  ofxPanel gui;

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
  int drawFluid_allow;
  double level_center;
  double level_left;
  double scanValues[726];
};

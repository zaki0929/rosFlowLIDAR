#pragma once

#include "ofMain.h"
#include "ofxFlowTools.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

using namespace flowTools;

class ofApp : public ofBaseApp{

public:
  void setup();
  void update();
  void draw();
  
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  void keyPressed(int key);
  void keyReleased(int key);

  ftFluidSimulation fluidSimulation;
  ftDrawMouseForces mouseForces;
  float deltaTime;
  float lastTime;
  int drawStar;

private:
  ros::NodeHandle n;
  ros::Subscriber scan_sub;
};

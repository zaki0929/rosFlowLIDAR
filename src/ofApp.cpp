#include "ofApp.h"
#define RANGE_MAX 5.6

void ofApp::setup(){
  scan_sub = n.subscribe("scan", 1000, &ofApp::scanCallback, this);
  ofBackground(0, 0, 0);
  ofSetWindowShape(1280, 720);
  fluidSimulation.setup(ofGetWidth(), ofGetHeight());
  fluidSimulation.setDissipation(0.0);
  mouseForces.setup(ofGetWidth(), ofGetHeight(),ofGetWidth(),ofGetHeight());
  lastTime = ofGetElapsedTimef();
}

double null_check(double target){
  if(!(target > 0)){
    target = (double)RANGE_MAX;
    ROS_WARN("RANGE OVER");
  }
  return target;
}

void ofApp::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

  double center_number = (-msg->angle_min)/msg->angle_increment;
  double center = msg->ranges[center_number];
  double left = msg->ranges[center_number+128];

  center = null_check(center);
  left = null_check(left);
  
  if(center < 0.5 || left < 0.5){
    ROS_INFO("Drawing");
    drawStar = 1;
    if(center < 0.5) level_center = center * 1000;
    if(left < 0.5) level_left = left * 1000;
  }else{
    drawStar = 0;
  }
  
  //ROS_INFO("center: [%lf]", center);
}

void ofApp::update(){
  deltaTime = ofGetElapsedTimef()-lastTime;
  lastTime = ofGetElapsedTimef();
  mouseForces.update(deltaTime);

  for (int i = 0; i<mouseForces.getNumForces(); i++) {
    if(mouseForces.didChange(i)){
      fluidSimulation.addDensity(mouseForces.getTextureReference(i),mouseForces.getStrength(i));
      fluidSimulation.addVelocity(mouseForces.getTextureReference(i),mouseForces.getStrength(i));
    }
  }
  fluidSimulation.update();
  
  ros::spinOnce();
}

void ofApp::draw(){
  fluidSimulation.draw(0,0,1280,720);
  if(drawStar){
    ofSetPolyMode(OF_POLY_WINDING_NONZERO);
    ofBeginShape();
    ofVertex(200+level_center,135+level_left);
    ofVertex(15+level_center,135+level_left);
    ofVertex(165+level_center,25+level_left);
    ofVertex(105+level_center,200+level_left);
    ofVertex(50+level_center,25+level_left);
    ofEndShape();
  }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
  if(key == 'c'){
    fluidSimulation.reset();
  }
}

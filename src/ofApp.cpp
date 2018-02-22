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

  center = null_check(center);
  
  if(center < 0.5){
    ROS_INFO("Drawing");
    drawStar = 1;
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
    ofVertex(200,135);
    ofVertex(15,135);
    ofVertex(165,25);
    ofVertex(105,200);
    ofVertex(50,25);
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

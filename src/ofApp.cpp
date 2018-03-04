#include "ofApp.h"
#define RANGE_MAX 5.6
#define VALID_MIN 44
#define VALID_MAX 725
#define ANGLE_DIFF 0.00613592315153
#define INITIAL_ANGLE 2.618 

void ofApp::setup(){
  scan_sub = n.subscribe("scan", 1000, &ofApp::scanCallback, this);
  ofBackground(0, 0, 0);
  ofSetWindowShape(1280, 720);

  fluidSimulation.setup(ofGetWidth(), ofGetHeight());
  fluidSimulation.setDissipation(0.0);

  mouseForces.setup(ofGetWidth(), ofGetHeight(),ofGetWidth(),ofGetHeight());
  lastTime = ofGetElapsedTimef();

  gui.setup();
  gui.add(mode.setup("mode", 3, 0, 4));
}

double null_check(double target){
  if(!(target > 0)){
    target = (double)RANGE_MAX;
    //ROS_WARN("RANGE OVER");
  }
  return target;
}

void ofApp::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  double center_number = (-msg->angle_min)/msg->angle_increment;
  double center = msg->ranges[center_number];
  double left = msg->ranges[center_number+128];

  center = null_check(center);
  left = null_check(left);

  //For draw the star.
  if(center < 0.5 || left < 0.5){
    drawStar_allow = 1;
    if(center < 0.5) level_center = center * 1000;
    if(left < 0.5) level_left = left * 1000;
  }else{
    drawStar_allow = 0;
  }

  //For draw the wave.
  for(int i=0; i<msg->ranges.size(); i++){
    scanValues[i] = null_check(msg->ranges[i]);
  }

  //For draw the fluid.
  if(center < 0.5){
    drawFluid_allow = 1;
  }else{
    drawFluid_allow = 0;
  }
}

void ofApp::update(){
  deltaTime = ofGetElapsedTimef()-lastTime;
  lastTime = ofGetElapsedTimef();
  mouseForces.update(deltaTime);

  for (int i=0; i<mouseForces.getNumForces(); i++) {
    if(mouseForces.didChange(i)){
      fluidSimulation.addDensity(mouseForces.getTextureReference(i),mouseForces.getStrength(i));
      fluidSimulation.addVelocity(mouseForces.getTextureReference(i),mouseForces.getStrength(i));
    }
  }
  fluidSimulation.update();
  
  ros::spinOnce();
}

void ofApp::draw(){
  //fluidSimulation.draw(0,0,1280,720);
  switch(mode){
    case 0:
      drawFluid();
      break;

    case 1:
      drawStar();
      break;

    case 2:
      drawWave();
      break;

    case 3:
      drawSun();
      break;

    case 4:
      drawSnow();
      break;
  }
  gui.draw();
}

void ofApp::drawStar(){
  if(drawStar_allow){
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

void ofApp::drawWave(){
  ofSetPolyMode(OF_POLY_WINDING_NONZERO);
  ofBeginShape();
  ofVertex(VALID_MIN, RANGE_MAX*100);
  for(int i=VALID_MIN; i<=VALID_MAX; i++){
    ofVertex(i, scanValues[i]*100);
  }
  ofVertex(VALID_MAX, RANGE_MAX*100);
  ofEndShape();
}

void ofApp::drawFluid(){
  if(drawFluid_allow){
    fluidSimulation.draw(0,0,1280,720);
  }
}

void ofApp::drawSun(){
  double angle = INITIAL_ANGLE;
  ofSetPolyMode(OF_POLY_WINDING_NONZERO);
  ofBeginShape();
  ofVertex(640, 360);
  for(int i=VALID_MIN; i<=VALID_MAX; i++){
    ofVertex(scanValues[i]*std::cos(angle)*60+640, -scanValues[i]*std::sin(angle)*60+360);
    angle += ANGLE_DIFF;
  }
  ofVertex(640, 360);
  angle = INITIAL_ANGLE;
  ofEndShape();
}

void ofApp::drawSnow(){
  double angle = INITIAL_ANGLE;
  for(int i=VALID_MIN; i<=VALID_MAX; i++){
    ofDrawCircle(scanValues[i]*std::cos(angle)*60+640, -scanValues[i]*std::sin(angle)*60+360, 0.5);
    angle += ANGLE_DIFF;
  }
  angle = INITIAL_ANGLE;
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

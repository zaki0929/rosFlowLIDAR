#include "ofMain.h"
#include "ofApp.h"
#include "ros/ros.h"

int main(int argc, char *argv[]){
  ros::init(argc, argv, "flow_lidar");
  ofSetupOpenGL(1024,768,OF_WINDOW);
  ofApp *app = new ofApp();
  ofRunApp(app);
}

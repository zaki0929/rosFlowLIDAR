PROJECT_LDFLAGS=-Wl,-rpath=./libs
PROJECT_LDFLAGS+=$(SUBLIBS) $(ros_libs_nocolon) 
ros_libs = $(shell pkg-config --libs roscpp sensor_msgs)
ros_libs_nocolon = $(subst -l:,,$(ros_libs))
PROJECT_OPTIMIZATION_CFLAGS_DEBUG = `pkg-config --cflags roscpp sensor_msgs` -w -O2 
PROJECT_OPTIMIZATION_CFLAGS_RELEASE = `pkg-config --cflags roscpp sensor_msgs` -w -O2 

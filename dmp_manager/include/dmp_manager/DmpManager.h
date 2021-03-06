#ifndef DMP_MANAGER_H
#define DMP_MANAGER_H

////////// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

////////// BOOST
#include <boost/shared_ptr.hpp>

////////// DMP
#include <dmp/Dmp.hpp>
#include <dmp/Trajectory.hpp>
#include <dynamicalsystems/DynamicalSystem.hpp>
#include <dynamicalsystems/ExponentialSystem.hpp>
#include <dynamicalsystems/SigmoidSystem.hpp>
#include <dynamicalsystems/TimeSystem.hpp>
#include <dynamicalsystems/SpringDamperSystem.hpp>
#include <functionapproximators/FunctionApproximatorLWR.hpp>
#include <functionapproximators/MetaParametersLWR.hpp>
#include <functionapproximators/ModelParametersLWR.hpp>

////////// ROS_DMP
#include <dmp_controller/DmpController.h>

////////// YAML
#include <yaml-cpp/yaml.h>

////////// STD
#include <fstream>
#include <signal.h> 

sig_atomic_t stop_node = 0;
void shutdown(int dummy) { stop_node = 1; }

#endif
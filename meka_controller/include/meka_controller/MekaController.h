/**
 * @file MekaController.h
 * @brief MekaController class header file.
 * @author Gennaro Raiola
 */

#ifndef MEKA_CONTROLLER_H
#define MEKA_CONTROLLER_H

////////// BOOST
#include <boost/thread/thread.hpp>

////////// STD
#include <atomic>

////////// ROS_DMP
#include <dmp_controller/DmpController.h>

////////// ROS
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

////////// MEKA SHARED MEMORY MONITOR
#include "meka_controller/MekaShmMonitor.h"

// The task will be scheduled as hard real time
//#define HARD_RT
// Modify the dmp integration period (useful if the meka is working at a lower frequency)
#define DT_FACTOR 1
// Debug
#define DEBUG 1


/** 
 * \brief This is an implementation of a dmp controller for the meka robot.
*/

//using namespace dmp_controller;

namespace meka_controller {

	class MekaController: public dmp_controller::DmpController 
	{
		
		public:
			/** Constructor. */
			MekaController();
			
			/** Destructor. */
			virtual ~MekaController();
			
			/** Initialization function, it works as constructor. */
			bool init(double dt, boost::shared_ptr<dmp_t> dmp_shr_ptr, bool cartesian_dmp_controller = false, bool closed_loop_dmp_controller = false);
			
			/** Start the controller. */
			void start();
			
			/** Update loop. */
			void updateLoop();
			
			/** Stop the controller. */
			void stop();
			
		protected:
			/** Read the joint positions. */
			void status();
			
			/** Write the joint positions. */
			void command();
			
			/** Initialize the real time task using rtai api. */
			bool rtTaskInit();
			
			/** Thread. */
			boost::thread thread_;
			
			/** Kill the thread. */
			std::atomic<bool> kill_thread_;
			
			/** Rtai stuff. */
			MekaShmMonitor* monitor_;
			RT_TASK* rt_task_;
	};
}

#endif
/**
 * @file DmpController.h
 * @brief DmpController class header file.
 * @author Gennaro Raiola
 */

#ifndef DMP_CONTROLLER_H
#define DMP_CONTROLLER_H

////////// BOOST
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

////////// DMP
#include <dmp/Dmp.hpp>

////////// Eigen
#include <eigen3/Eigen/Core>

////////// Debug
#define PRINT_DEBUG(string,in) do { if (DEBUG) std::cout<<string<<in<<std::endl; } while (0)


/** 
 * \brief This class provides a general interface for dmp controllers.
*/

using namespace DmpBbo;

typedef Dmp dmp_t;

namespace dmp_controller{

	class DmpController
	{	
		public:
			/** Constructor. */
			//DmpController(double dt,boost::shared_ptr<dmp_t> dmp_shr_ptr);
			DmpController(){}; //Note: this needs to be an empty constructor for the ros plugin creation
			
			/** Destructor. */
			virtual ~DmpController(){};
			
			/** Initialization function, it works as constructor. */
			virtual bool init(double dt, boost::shared_ptr<dmp_t> dmp_shr_ptr, bool cartesian_dmp_controller = false, bool closed_loop_dmp_controller = false)
			{
				// Is it a cartesian dmp controller, closed loop?
				cartesian_dmp_controller_ = cartesian_dmp_controller;
				closed_loop_dmp_controller_ = closed_loop_dmp_controller;
				
				// Assign the sample time
				assert(dt > 0.0);
				dt_ = dt;
				
				// Check if the dmp is trained
				//if(!dmp_shr_ptr.isTrained())
				//	return false;
				dmp_shr_ptr_ = dmp_shr_ptr;
				
				// Resize the dmp's state vectors
				dmp_state_size_ = dmp_shr_ptr_->dim();
				dmp_state_status_.resize(dmp_state_size_);
				dmp_state_status_dot_.resize(dmp_state_size_);
				dmp_state_command_.resize(dmp_state_size_);
				dmp_state_command_dot_.resize(dmp_state_size_);
				
				if (cartesian_dmp_controller_){
					// Retrain a kdl robot
					
					//joints_size_ = 7;
					//assert(joints_size_ == dmp_shr_ptr_->dim_orig());
				}
				else
					joints_size_ = dmp_shr_ptr_->dim_orig();
					
				// Resize the joints vectors
				joints_status_.resize(joints_size_);
				joints_status_dot_.resize(joints_size_);
				joints_command_.resize(joints_size_);
				joints_command_dot_.resize(joints_size_);
				
				// Prepare the state vectors to be integrated by dmp
				dmp_shr_ptr_->integrateStart(dmp_state_status_,dmp_state_status_dot_);
				
				return true;
			}
			
			/** Start the controller. */
			virtual void start() = 0;
			
			/** Update loop. */
			virtual void updateLoop() = 0;
			
			/** Update step, should be looped inside updateLoop. */
			void updateStep()
			{
				// Read the joints state
				status(); // Update joints_status_ and joints_status_dot_
		
				// Dmp's magic
				if (cartesian_dmp_controller_){
					//CrtDmpIntegrate(joints_status_,joints_status_dot_,joints_command_,joints_command_dot_);
				}
				else
					JntDmpIntegrate();
				
				// Write the joints commands
				command(); // Use joints_command_ and joints_command_dot_
			}
			
			/** Stop the controller. */
			virtual void stop() = 0;
			
		protected:
			
			/** Sample time. */
			double dt_;
			
			/** Number of joints controlled. */
			int joints_size_;
			
			/** Dmp's state size, can be equal to the number of joints. */
			int dmp_state_size_;
			
			/** Define if it's a cartesian dmp controller or not. */
			bool cartesian_dmp_controller_;
			
			/** Define if it's a closed loop dmp controller or not. */
			bool closed_loop_dmp_controller_;
			
			/** Shared pointer to a trained dmp. */
			boost::shared_ptr<dmp_t> dmp_shr_ptr_;
			
			/** Thread. */
			//boost::thread thread_;
			
			/** Pre-allocated attributes. */
			/** Joint vectors. */
			Eigen::VectorXd joints_status_;
			Eigen::VectorXd joints_status_dot_;
			Eigen::VectorXd joints_command_;
			Eigen::VectorXd joints_command_dot_;
			/** Dmp state vectors. */
			Eigen::VectorXd dmp_state_status_;
			Eigen::VectorXd dmp_state_status_dot_;
			Eigen::VectorXd dmp_state_command_;
			Eigen::VectorXd dmp_state_command_dot_;
			
			/** Read the joint positions. */
			virtual void status() = 0;
			
			/** Write the joint positions. */
			virtual void command() = 0;
			
			/** Integrate dmp in joints space. */
			inline void JntDmpIntegrate()
			{
				if(closed_loop_dmp_controller_){
					dmp_state_status_.segment(0,joints_size_) = joints_status_; // Pos
					dmp_state_status_.segment(joints_size_,joints_size_) = joints_status_dot_; // Vel
				}
		
				dmp_shr_ptr_->integrateStep(dt_,dmp_state_status_,dmp_state_command_,dmp_state_command_dot_);
				// Update the gating system state
				dmp_state_status_ =  dmp_state_command_;
		
				joints_command_ = dmp_state_command_.segment(0,joints_size_); // Pos
				joints_command_dot_ = dmp_state_command_.segment(joints_size_,joints_size_); // Vel;
			}
			
			/** Integrate dmp in cartesian space. */
			void CrtDmpIntegrate();
			
			

	};

}

#endif
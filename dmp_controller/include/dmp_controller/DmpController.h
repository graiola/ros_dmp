/**
 * @file DmpController.h
 * @brief DmpController class header file.
 * @author Gennaro Raiola
 */

#ifndef DMP_CONTROLLER_H
#define DMP_CONTROLLER_H

////////// KDL_KINEMATICS
#include <kdl_kinematics/kdl_kinematics.h>

////////// BOOST
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

////////// DMP
#include <dmp/Dmp.hpp>

////////// Eigen
#include <eigen3/Eigen/Core>

////////// Debug
#ifdef DEBUG
#define DEBUG_LV 1
#else
#define DEBUG_LV 0
#endif
#define PRINT_DEBUG(string,in) do { if (DEBUG_LV) std::cout<<string<<in<<std::endl; } while (0)


/** 
 * \brief This class provides a general interface for dmp controllers.
*/

typedef DmpBbo::Dmp dmp_t;
typedef kdl_kinematics::KDLKinematics kinematics_t;

namespace dmp_controller {

	class DmpController
	{	
		public:
			/** Constructor. */
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
				
				// Create the kinematics
				if (cartesian_dmp_controller_)// FIX, hardcoded
				{
					// Retrain a kdl robot
					kinematics_ = boost::make_shared<kinematics_t> ("T0","palm_right");
					if(!kinematics_->isParsed()){
						ROS_ERROR("Problem with kdl_kinematics"); // FIX, handle it with try catch, implement a throw inside kdl_kinematics
						return false;
					}
					
					// Check if dmp has xyz rpy
					assert(dmp_shr_ptr_->dim_orig() == 6); // FIX, hardcoded, I should implement a kind of mask...
					cart_size_ = dmp_shr_ptr_->dim_orig();
					
					// Resize the IK attributes
					v_.resize(cart_size_);
					//gain_.resize(cart_size_,cart_size_); // FIX
					// Assign the gains
					gain_ = 100;
					
					joints_size_ = kinematics_->getNdof();
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
				if (cartesian_dmp_controller_)
					CrtDmpIntegrate();
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
			
			/** Number of dof controlled in the cartesian space. */
			int cart_size_;
			
			/** Dmp's state size, can be equal to the number of joints or to the number of cartesian dof. */
			int dmp_state_size_;
			
			/** Define if it's a cartesian dmp controller or not. */
			bool cartesian_dmp_controller_;
			
			/** Define if it's a closed loop dmp controller or not. */
			bool closed_loop_dmp_controller_;
			
			/** Shared pointer to a trained dmp. */
			boost::shared_ptr<dmp_t> dmp_shr_ptr_;
			
			/** Kinematics. */
			boost::shared_ptr<kinematics_t> kinematics_;
			
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
			/** IK attributes */
			Eigen::VectorXd v_;
			//Eigen::MatrixXd gain_; // FIX
			double gain_;
			
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
				joints_command_dot_ = dmp_state_command_.segment(joints_size_,joints_size_); // Vel
			}
			
			/** Integrate dmp in cartesian space. */
			inline void CrtDmpIntegrate()
			{
				
				
				
				kinematics_->ComputeFk(joints_status_,dmp_state_status_.segment(0,cart_size_));//FIX, xyz rpy
				
				dmp_shr_ptr_->integrateStep(dt_,dmp_state_status_,dmp_state_command_,dmp_state_command_dot_);
				
				v_ = gain_*(dmp_state_command_.segment(0,cart_size_) - dmp_state_status_.segment(0,cart_size_)) + dmp_state_command_.segment(cart_size_,cart_size_);
				
				kinematics_->ComputeIk(joints_status_,v_,joints_command_dot_);
				
				// Update the gating system state
				dmp_state_status_ =  dmp_state_command_;
				
				joints_command_ = dt_ * joints_command_dot_ +  joints_status_; // Pos computed with euler integration
				
			}
			
			

	};

}

#endif
/**
 * @file DmpController.h
 * @brief DmpController class header file.
 * @author Gennaro Raiola
 */

#ifndef DMP_CONTROLLER_H
#define DMP_CONTROLLER_H

////////// ROS
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <realtime_tools/realtime_publisher.h>

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
#define PRINT_DEBUG(lv,string,in) do { if (DEBUG_LV == lv) std::cout<<string<<in<<std::endl; } while (0)


/** 
 * \brief This class provides a general interface for dmp controllers.
*/

typedef DmpBbo::Dmp dmp_t;
typedef kdl_kinematics::KDLKinematics kinematics_t; 
typedef realtime_tools::RealtimePublisher<sensor_msgs::JointState> rt_joints_publisher_t;
typedef realtime_tools::RealtimePublisher<geometry_msgs::PoseArray> rt_cart_publisher_t;

namespace dmp_controller {

	class DmpController
	{	
		public:
			/** Constructor. */
			DmpController(){}; //Note: this needs to be an empty constructor for the ros plugin creation
			
			/** Destructor. */
			virtual ~DmpController(){};
			
			/** Initialization function, it works as constructor. */
			virtual bool init(ros::NodeHandle& ros_nh, double dt, boost::shared_ptr<dmp_t> dmp_shr_ptr, bool cartesian_dmp_controller = false, bool closed_loop_dmp_controller = false)
			{
				// Take the ros node handle for this controller
				ros_nh_ = ros_nh;	
				
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
					try
					{
						kinematics_ = boost::make_shared<kinematics_t> ("T0","palm_right"); // Fix hardcoded names
					}
					catch(const std::runtime_error& e)
					{
						std::cout << e.what() << std::endl;
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
				
				// Initialize the real time publishers
				jointsPublisherInit(joints_status_pub_shr_ptr_,"joints_status_dmp", ros_nh_);	
				jointsPublisherInit(joints_cmd_pub_shr_ptr_,"joints_cmd_dmp", ros_nh_);
				jointsPublisherInit(joints_error_pub_shr_ptr_,"joints_err_dmp", ros_nh_);
					
				// Resize the joints vectors
				joints_status_.resize(joints_size_);
				joints_status_dot_.resize(joints_size_);
				joints_command_.resize(joints_size_);
				joints_command_dot_.resize(joints_size_);
				joints_error_.resize(joints_size_);
				
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
		
				// Integrate dmp
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
			
			/** Ros node handle. */
			ros::NodeHandle ros_nh_;
			
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
			
			/** Real time publishers. */
			boost::shared_ptr<rt_joints_publisher_t > joints_status_pub_shr_ptr_;
			boost::shared_ptr<rt_joints_publisher_t > joints_cmd_pub_shr_ptr_;
			boost::shared_ptr<rt_joints_publisher_t > joints_error_pub_shr_ptr_;
			boost::shared_ptr<rt_cart_publisher_t > cart_publisher_shr_ptr_;
			
			/** Thread. */
			//boost::thread thread_;
			
			/** Pre-allocated attributes. */
			/** Joint vectors. */
			Eigen::VectorXd joints_status_;
			Eigen::VectorXd joints_status_dot_;
			Eigen::VectorXd joints_command_;
			Eigen::VectorXd joints_command_dot_;
			Eigen::VectorXd joints_error_;
			/** Dmp state vectors. */
			Eigen::VectorXd dmp_state_status_;
			Eigen::VectorXd dmp_state_status_dot_;
			Eigen::VectorXd dmp_state_command_;
			Eigen::VectorXd dmp_state_command_dot_;
			/** IK attributes */
			Eigen::VectorXd v_;
			//Eigen::MatrixXd gain_; // FIX
			double gain_;
			
			/** Get the the joints status. */
			inline void status()
			{
				readJointsStatus();
				PRINT_DEBUG(1,"joints_status_\n",joints_status_);
				PRINT_DEBUG(1,"dmp_state_status_\n",dmp_state_status_);
				// Compute the error on the joints using the previous commands
				joints_error_ = joints_command_ - joints_status_;
				publishJoints(joints_status_pub_shr_ptr_,joints_status_);
				publishJoints(joints_error_pub_shr_ptr_,joints_error_);
			}
			
			/** Send the joints commands. */
			inline void command()
			{
				writeJointsCommands();
				PRINT_DEBUG(1,"dmp_state_command_\n",dmp_state_command_);
				PRINT_DEBUG(1,"joints_command_\n",joints_command_);
				publishJoints(joints_cmd_pub_shr_ptr_,joints_command_);
			}
			
			/** Read from motors (robot dependent). */
			virtual void readJointsStatus() = 0;
			
			/** Write to motors (robot dependent). */
			virtual void writeJointsCommands() = 0;
			
			/** Initialize the real time publisher. */
			void jointsPublisherInit(boost::shared_ptr<rt_joints_publisher_t >& pub_ptr,std::string topic_name, ros::NodeHandle& ros_nh)
			{	assert(joints_size_ > 0);
				pub_ptr.reset(new rt_joints_publisher_t(ros_nh,topic_name,10));
				for(int i = 0; i < joints_size_; i++){
					pub_ptr->msg_.name.push_back("joint_"+std::to_string(i));
					pub_ptr->msg_.position.push_back(0.0);
					//pub_ptr->msg_.velocity.push_back(0.0);
				}
			}
			
			/** Publish the joints position. */
			inline void publishJoints(boost::shared_ptr<rt_joints_publisher_t >& pub_ptr, Eigen::Ref<Eigen::VectorXd> joints_pos)
			{
				if(pub_ptr && pub_ptr->trylock())
				{
					pub_ptr->msg_.header.stamp = ros::Time::now();
					for(int i = 0; i < joints_size_; i++)
					{
						pub_ptr->msg_.position[i] = joints_pos[i];
						//pub_ptr->msg_.velocity[i] = joints_vel[i];
					}
					pub_ptr->unlockAndPublish();
				}
			}	
			/** Publish the cartesian position and the velocity. */
			/*inline void publishCart(boost::shared_ptr<rt_cart_publisher_t > pub_ptr, Eigen::Ref<Eigen::VectorXd> cart_pos)
			{
				if(pub_ptr->trylock())
				{
					pub_ptr->msg_.header.stamp = ros::Time::now();
					for(int i = 0; i < cart_size_; i++)
						pub_ptr->msg_.poses[i] = cart_pos[i];
					pub_ptr->unlockAndPublish();
				}	
			}*/
			
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
				if(closed_loop_dmp_controller_)
				{
					kinematics_->ComputeFk(joints_status_,dmp_state_status_.segment(0,cart_size_));//FIX, xyz rpy
					dmp_shr_ptr_->integrateStep(dt_,dmp_state_status_,dmp_state_command_,dmp_state_command_dot_);
				}
				else
				{
					dmp_shr_ptr_->integrateStep(dt_,dmp_state_status_,dmp_state_command_,dmp_state_command_dot_);
					kinematics_->ComputeFk(joints_status_,dmp_state_status_.segment(0,cart_size_));//FIX, xyz rpy
				}
				
				v_ = gain_*(dmp_state_command_.segment(0,cart_size_) - dmp_state_status_.segment(0,cart_size_)) + dmp_state_command_.segment(cart_size_,cart_size_); // The ik is always working in closed loop
				
				kinematics_->ComputeIk(joints_status_,v_,joints_command_dot_);
				
				// Update the gating system state
				dmp_state_status_ =  dmp_state_command_;
				
				joints_command_ = dt_ * joints_command_dot_ +  joints_status_; // Pos computed with euler integration
			}
			
			

	};

}

#endif
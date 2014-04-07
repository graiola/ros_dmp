#include "meka_controller/MekaController.h"

namespace meka_controller{
	
	MekaController::MekaController()
	{
	}
	
	MekaController::~MekaController()
	{
		delete monitor_;
	}
	
	bool MekaController::init(ros::NodeHandle& ros_nh, double dt, boost::shared_ptr<dmp_t> dmp_shr_ptr, kinematics_t* kin_ptr, bool closed_loop_dmp_controller)
	{	
		if(dmp_controller::DmpController::init(ros_nh,dt,dmp_shr_ptr,kin_ptr,closed_loop_dmp_controller))
		{
			monitor_ = new MekaShmMonitor(joints_size_);
			return true;
		}
		else
		{	ROS_ERROR("Cannot initialize the meka controller");
			return false;
		}
	}
	
	bool MekaController::rtTaskInit()
	{
		rt_allow_nonroot_hrt();							
		//Args: Name, Priority, Stack Size, max_msg_size, Policy, cpus_allowed
		if (!(rt_task_ = rt_task_init_schmod(nam2num( "M3DMP" ),0,0,0,SCHED_FIFO,0xF)))
		{	
			ROS_ERROR("Cannot initialize the real time task for the meka_controller");
			return false;
		}
		if(!monitor_->m3sdsInit(BOT_SHM)){ // FIX, the hardcoded names
			ROS_ERROR("Unable to find the %s shared memory.\n",BOT_SHM);
			rt_task_delete(rt_task_);
			return false;
		}
		if(!monitor_->statusSemInit(BOT_STATUS_SEM)){
			ROS_ERROR("Unable to find the %s semaphore.\n",BOT_STATUS_SEM);
			rt_task_delete(rt_task_);
			return false;
		}
		if(!monitor_->commandSemInit(BOT_CMD_SEM)){
			ROS_ERROR("Unable to find the %s semaphore.\n",BOT_CMD_SEM);
			rt_task_delete(rt_task_);
			return false;
		}
		
		// Set the task period
		RTIME tick_period = nano2count(SEC2NANO(dt_)); // This is ~=dt;
		rt_task_make_periodic(rt_task_, rt_get_time() + tick_period, tick_period);
		mlockall(MCL_CURRENT | MCL_FUTURE); // Prevent memory swaps
		
#ifdef HARD_RT
		rt_make_hard_real_time();
#endif
		return true;
	}

	void MekaController::updateLoop()
	{
		INIT_CNT(tmp_dt_cnt);
		INIT_CNT(tmp_loop_cnt);
		if(rtTaskInit()){
			SAVE_TIME(start_loop_time);
			while(!kill_thread_)
			{
				SAVE_TIME(start_dt_time);
				dmp_controller::DmpController::updateStep(); // This will loop
				rt_task_wait_period(); // Wait until the end of the period.
				SAVE_TIME(end_dt_time);
				PRINT_TIME(start_dt_time,end_dt_time,tmp_dt_cnt,"dt");
			}
			SAVE_TIME(end_loop_time);
			PRINT_TIME(start_loop_time,end_loop_time,tmp_loop_cnt,"elapsed time");
			rt_task_delete(rt_task_);
		}
	}
	
	void MekaController::readJointsStatus()
	{
		monitor_->stepStatus(joints_status_);	
	}
	
	void MekaController::writeJointsCommands()
	{
		monitor_->stepCommand(joints_command_);
	}
	
	void MekaController::start()
	{
		ROS_INFO("Starting the thread");
		kill_thread_ = false;
		try
		{
			thread_ = boost::thread(&MekaController::updateLoop, this); 
		}
		catch(boost::thread_resource_error &e)
		{
			ROS_ERROR_STREAM("exception at MekaController::start(), can not create the thread, reason: "<< e.what());
		}
}
	
	void MekaController::stop()
	{
		ROS_INFO("Stopping the thread");
		kill_thread_ = true;
		thread_.join();
	}

//Declare MekaController as a DmpController class
PLUGINLIB_EXPORT_CLASS(meka_controller::MekaController, dmp_controller::DmpController);
}
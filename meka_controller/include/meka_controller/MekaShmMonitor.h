#ifndef MEKA_SHM_MONITOR_H
#define MEKA_SHM_MONITOR_H

////////// RT/M3
#include <rtai_sched.h>
#include <stdio.h>
#include <signal.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>
#include <m3rt/base/m3rt_def.h>
#include <rtai_nam2num.h>
#include <rtai_registry.h>
#include <eigen3/Eigen/Core>
#include <m3/robots/humanoid_shm_sds.h>

////////// SOME RT MACROS AND DEFINES
#define NANO2SEC(a)	a/1e9
#define SEC2NANO(a)	a*1e9
#define BOT_SHM "TSHMM"
#define BOT_CMD_SEM "TSHMC"
#define BOT_STATUS_SEM "TSHMS"
	
////////// Activate some timing infos
//#define TIMING
static int tmp_dt_cnt;
static int tmp_loop_cnt;
static long long start_dt_time, end_dt_time, elapsed_dt_time;
static long long start_loop_time, end_loop_time, elapsed_loop_time;

#ifdef TIMING
#define TIME_ACTIVE 1
#else
#define TIME_ACTIVE 0
#endif

#define INIT_CNT(cnt) do { if (TIME_ACTIVE) (cnt) = 0; } while (0) 
#define SAVE_TIME(out) do { if (TIME_ACTIVE) getCpuCount((out)); } while (0)
#define PRINT_TIME(T_start,T_end,cnt,string) do { if (TIME_ACTIVE) if ((cnt)%100==0) ROS_INFO("%s: %fs",string,count2Sec(((T_end) - (T_start)))); cnt = cnt++ & INT_MAX;} while (0)

using namespace Eigen;
using namespace std;

void getCpuCount(long long& out){
	out = nano2count(rt_get_cpu_time_ns());
}

double count2Sec(const long long in){
	return (NANO2SEC((double)count2nano(in)));
}

bool getSemAddr(const char* sem_name,SEM* &sem){
	sem = (SEM*)rt_get_adr(nam2num(sem_name));
	if (!sem)
		return false;
	return true;
}

class MekaShmMonitor
{
	private:
		int Ndof_;
		int sds_status_size_;
		int sds_cmd_size_;
		M3HumanoidShmSdsStatus status_;
		M3HumanoidShmSdsCommand cmd_;
		SEM* status_sem_;
		SEM* command_sem_;
		M3Sds* m3_sds_;
		std::string bot_shm_name_;
	public:
		MekaShmMonitor(int Ndof):Ndof_(Ndof),sds_status_size_(sizeof(M3HumanoidShmSdsStatus)),sds_cmd_size_(sizeof(M3HumanoidShmSdsCommand)){
			memset(&cmd_, 0, sds_cmd_size_); // Initialize cmd
			memset(&status_, 0, sds_status_size_); // Initialize status
		}
		
		~MekaShmMonitor(){
			rt_shm_free(nam2num(bot_shm_name_.c_str()));

		}
		
		bool statusSemInit(std::string sem_name){
			if(getSemAddr(sem_name.c_str(),status_sem_))
				return true;
			else
				return false;
		}
		bool commandSemInit(std::string sem_name){
			if(getSemAddr(sem_name.c_str(),command_sem_))
				return true;
			else
				return false;
		}
		
		bool m3sdsInit(std::string bot_shm_name){
			if ((this->m3_sds_ = (M3Sds*)rt_shm_alloc(nam2num(bot_shm_name.c_str()),sizeof(M3Sds),USE_VMALLOC))){
				bot_shm_name_ = bot_shm_name; 
				return true;
			}
			else
				return false;
		}
		
		inline void stepCommand(VectorXd& joints_cmd)
		{	
			assert(joints_cmd.size() >= Ndof_);
			setTimestamp(getTimestamp()); //Pass back timestamp as a heartbeat
			for (int i = 0; i < Ndof_; i++)
			{	//FIX, Hardcoded values
				cmd_.right_arm.ctrl_mode[i] = JOINT_ARRAY_MODE_THETA_GC;
				cmd_.right_arm.q_desired[i] = (mReal)joints_cmd[i];
				cmd_.right_arm.tq_desired[i] = 40.0;
				cmd_.right_arm.slew_rate_q_desired[i] = 10.0; // 10.0
				cmd_.right_arm.q_stiffness[i] = 1.0;
			}
			// Lock the semaphore and copy the output data
			rt_sem_wait(command_sem_);
			memcpy(this->m3_sds_->cmd, &cmd_, sds_cmd_size_);
			rt_sem_signal(command_sem_);
		}

		inline void stepStatus(VectorXd& joints_status)
		{	
			assert(joints_status.size() >= Ndof_);
			// Lock the semaphore and copy the input data
			rt_sem_wait(status_sem_);
			memcpy(&status_, this->m3_sds_->status, sds_status_size_);
			rt_sem_signal(status_sem_);
			// Convert status into Eigen vector
			for (int i = 0; i < Ndof_; i++) // FIX
			{
				joints_status[i] = DEG2RAD(status_.right_arm.theta[i]); // pos
				//joints_status[i+7] = DEG2RAD(status.right_arm.thetadot[i]); // vel
			}
		}	
		
		///////  Timing functions:
		void setTimestamp(int64_t  timestamp)
		{
			cmd_.timestamp = timestamp;
			return;
		}

		int64_t getTimestamp()
		{
			return status_.timestamp;
		}
};

#endif
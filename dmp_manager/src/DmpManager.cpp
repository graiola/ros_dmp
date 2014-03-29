#include "dmp_manager/DmpManager.h"

//using namespace dmp_controller;

// FIX, this is what I should do, for now this .cpp is working as a test unit
/*class DmpManager
{
	public:
		pluginlib::ClassLoader<dmp_controller::DmpController> dmp_controller_loader; 
		
		boost::shared_ptr<dmp_controller::DmpController> controller; // I can create a map of controllers
	
};*/

using namespace DmpBbo;
using namespace Eigen;

Trajectory generateViapointTrajectory(const VectorXd& ts, const VectorXd& y_first, const VectorXd& y_last, const double& Tf, const double& Ti)
{
    //VectorXd y_first = VectorXd::Zero(n_dims);
    //VectorXd y_last  = VectorXd::Ones(n_dims) * 0.3;
    
    assert(y_first.size() == y_last.size());
    
    int n_dims = y_first.size();
    double viapoint_time = (Tf -Ti)/2;

    VectorXd y_yd_ydd_viapoint = VectorXd::Zero(3*n_dims);
    
    for(int i = 0; i<n_dims; i++)
	    y_yd_ydd_viapoint[i] = (y_last[i] - y_first[i])/2;
    
    return  Trajectory::generatePolynomialTrajectoryThroughViapoint(ts,y_first,y_yd_ydd_viapoint,viapoint_time,y_last);
}

dmp_t* generateDemoDmpJoints(const double dt, const int Ndof, const double Ti, const double Tf, int& n_time_steps_trajectory){

	// GENERATE A TRAJECTORY
	n_time_steps_trajectory = (int)((Tf-Ti)/dt) + 1;

	// Some default values for dynamical system
	VectorXd y_init = VectorXd::Zero(Ndof);
	VectorXd y_attr  = VectorXd::Ones(Ndof) * 0.4;
	
	VectorXd ts = VectorXd::LinSpaced(n_time_steps_trajectory,Ti,Tf); // From Ti to Tf in n_time_steps_trajectory steps
	
	Trajectory trajectory = generateViapointTrajectory(ts, y_init, y_attr, Tf, Ti);
  
        // MAKE THE FUNCTION APPROXIMATORS
        // Initialize some meta parameters for training LWR function approximator
        int n_basis_functions = 25;
        int input_dim = 1;
        double overlap = 0.01;
        MetaParametersLWR* meta_parameters = new MetaParametersLWR(input_dim,n_basis_functions,overlap);
        FunctionApproximatorLWR* fa_lwr = new FunctionApproximatorLWR(meta_parameters);

	//Dmp::DmpType dmp_type = Dmp::KULVICIUS_2012_JOINING;
	dmp_t::DmpType dmp_type = dmp_t::IJSPEERT_2002_MOVEMENT;
	
	std::vector<FunctionApproximator*> function_approximators(Ndof);    	
	for (int dd=0; dd<Ndof; dd++)
		function_approximators[dd] = fa_lwr->clone();
	
	  /*Dmp(double tau, Eigen::VectorXd y_init, Eigen::VectorXd y_attr, std::vector<FunctionApproximator*> function_approximators, DmpType dmp_type=KULVICIUS_2012_JOINING);*/
	
	dmp_t* dmp = new dmp_t(Tf,y_init,y_attr,function_approximators,dmp_type);
	
	dmp->train(trajectory);
	
	return dmp;  
}

int main(int argc, char *argv[])
{
	//ros::init(argc, argv, "dmp_manager");
	
	signal(SIGINT, shutdown);
	
	ros::init(argc, argv, "dmp_manager",ros::init_options::NoSigintHandler);
	
	pluginlib::ClassLoader<dmp_controller::DmpController> dmp_controller_loader("dmp_controller", "dmp_controller::DmpController");
	
	boost::shared_ptr<dmp_controller::DmpController> controller;
	
	try
	{
		controller = dmp_controller_loader.createInstance("meka_controller::MekaController");
		
	}
	catch(pluginlib::PluginlibException& ex)
	{
		//handle the class failing to load
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
		return 0;
	}

	// Dmp
	double Tf = 6; //sec
	double Ti = 0.0;
	double dt = 0.025;
	int Ndof = 7;
	int n_time_steps_trajectory;
	//Dmp* dmp_ptr = ;
	boost::shared_ptr<dmp_t> dmp_shr_ptr(generateDemoDmpJoints(dt,Ndof,Ti,Tf,n_time_steps_trajectory));
	
	if(controller->init(dt,dmp_shr_ptr)){
		controller->start();
		while (!stop_node){ // Wait for the kill signal
			usleep(200);
		}	
	}
	
	
	// End of the world
	controller->stop();
	ros::shutdown();
	return 0;
}

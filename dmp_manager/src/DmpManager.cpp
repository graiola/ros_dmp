#include "dmp_manager/DmpManager.h"
#include "dmp_manager/Kinematics.h"

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

std::string controller_name, mask;
bool closed_loop, cart_dmp;
double damp_max, epsilon;
std::vector<double> gains_vector(6);
MatrixXd ik_gain;

bool readConfig(std::string file_path)
{
	YAML::Node main_node;
	std::ifstream file(file_path.c_str());
	if(file.fail())
		return false;
	YAML::Parser parser(file);
	parser.GetNextDocument(main_node);
	file.close();
	
	main_node["controller"] >> controller_name;
	
	const YAML::Node& dmp_node = main_node["dmp"];
	dmp_node["closed_loop"] >> closed_loop;
	dmp_node["cartesian"] >> cart_dmp;
	
	const YAML::Node& dmp_ik_node = dmp_node["ik_gain"];
	dmp_ik_node["x"] >> gains_vector[0];
	dmp_ik_node["y"] >> gains_vector[1];
	dmp_ik_node["z"] >> gains_vector[2];
	dmp_ik_node["roll"] >> gains_vector[3];
	dmp_ik_node["pitch"] >> gains_vector[4];
	dmp_ik_node["yaw"] >> gains_vector[5];
	// Create the Eigen gain matrix
	//ik_gain.resize(6,6);
	ik_gain = MatrixXd::Zero(6,6);
	for(int i = 0; i<6; i++)
		ik_gain(i,i) = gains_vector[i];

	const YAML::Node& ik_node = main_node["ik"];
	ik_node["damp_max"] >> damp_max;
	ik_node["epsilon"] >> epsilon;
	ik_node["mask"] >> mask;
	
	return true;
}

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

dmp_t* generateDemoDmp(const VectorXd y_init, const VectorXd y_attr, const double dt, const int Ndof, const double Ti, const double Tf, int& n_time_steps_trajectory){

	assert(y_init.size() == Ndof);
	assert(y_attr.size() == Ndof);
	
	// GENERATE A TRAJECTORY
	n_time_steps_trajectory = (int)((Tf-Ti)/dt) + 1;
	
	// Some default values for dynamical system
	//VectorXd y_init = VectorXd::Zero(Ndof);
	//VectorXd y_attr  = VectorXd::Ones(Ndof) * 0.4;
	
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
	
	std::string file_path("/home/meka/catkin_workspace/src/ros_dmp/dmp_manager/config/config.yaml");
	if(readConfig(file_path))
		ROS_INFO("Loaded config file: %s",file_path.c_str());
	else
		ROS_ERROR("Can not load config file: %s",file_path.c_str());

	// Dmp
	double Tf = 6; //sec
	double Ti = 0.0;
	double dt = 0.025;
	int n_time_steps_trajectory;
	int Ndof;
	//bool cart_dmp = true;
	//bool closed_loop = true;
	VectorXd y_attr;
	if(cart_dmp){
		Ndof = 6;
		y_attr.resize(Ndof);
		y_attr << 0.24, -0.1, -0.53, 0.0, 0.0, 0.0; // Cart
	}
	else{
		Ndof = 7;
		y_attr  = VectorXd::Ones(Ndof) * 0.4; // Joints
	}
	
	VectorXd y_init = VectorXd::Zero(Ndof);
	
	ros::NodeHandle controller_nh("meka_controller");
	
	// Generate and train a DMP
	boost::shared_ptr<dmp_t> dmp_shr_ptr(generateDemoDmp(y_init,y_attr,dt,Ndof,Ti,Tf,n_time_steps_trajectory));
	
	// Retrain a kdl robot
	kinematics_t* kin_ptr = nullptr;
	if(cart_dmp)
	{
		try
		{
			kin_ptr = new kinematics_t("T0","palm_right",damp_max,epsilon,ik_gain); // Fix hardcoded names
			kin_ptr->setMask(mask); // FIX, now should be called ALWAYS before the dmp_controller initialization!
		}
		catch(const std::runtime_error& e)
		{
			std::cout << e.what() << std::endl;
			return 0;
		}
	}
	
	if(controller->init(controller_nh,dt,dmp_shr_ptr,kin_ptr,closed_loop)){
		//controller->gain_ = ik_gain; // HACK
		controller->start();
		while (!stop_node){ // Wait for the kill signal
			usleep(200);
		}	
	}
	
	// End of the world
	controller->stop();
	delete kin_ptr;
	ros::shutdown();
	return 0;
}

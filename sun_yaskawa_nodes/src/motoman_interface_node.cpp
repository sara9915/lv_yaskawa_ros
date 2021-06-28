#include "ros/ros.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"


#define NUM_JOINTS 7

using namespace std;

vector<string> default_joint_names;

/* Callback for joint state.*/
std::vector<double> q_read;
bool joint_states_arrived = false;
void readJointPos(const sensor_msgs::JointStateConstPtr &jointStateMsg)
{
	if (jointStateMsg->name.size() == NUM_JOINTS)
	{
		q_read = jointStateMsg->position;
		joint_states_arrived = true;
	}
}

/* Callback for joint command.*/
sensor_msgs::JointState jointCommand;
void readCommand(const sensor_msgs::JointStateConstPtr& jointCommandMsg)
{
	jointCommand = *jointCommandMsg;
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "motoman_interface");

	if (!ros::master::check())
		return (0);

	ros::NodeHandle nh;

	/* Publisher.*/
	ros::Publisher joint_cmd_pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_command", 1);

	/* Subscriber.*/
	ros::Subscriber ll_control_sub = nh.subscribe("joint_ll_control", 1, readCommand);

	float rate = 50.0; /* 50 Hz */
	ros::Rate loop_rate(rate);

	/* NODE inizialization.*/
	trajectory_msgs::JointTrajectoryPoint jointTrajPoint;
	trajectory_msgs::JointTrajectory jointTraj;
	jointTrajPoint.positions.resize(NUM_JOINTS);
	jointTrajPoint.velocities.resize(NUM_JOINTS);
	jointTraj.points.resize(1);

	ROS_INFO_STREAM("Starting motion command.");

	if (NUM_JOINTS == 6)
	{
		default_joint_names.push_back("joint_s");
		default_joint_names.push_back("joint_l");
		default_joint_names.push_back("joint_u");
		default_joint_names.push_back("joint_r");
		default_joint_names.push_back("joint_b");
		default_joint_names.push_back("joint_t");
	}
	else if (NUM_JOINTS == 7)
	{
		default_joint_names.push_back("joint_s");
		default_joint_names.push_back("joint_l");
		default_joint_names.push_back("joint_e");
		default_joint_names.push_back("joint_u");
		default_joint_names.push_back("joint_r");
		default_joint_names.push_back("joint_b");
		default_joint_names.push_back("joint_t");
	}

	//Wait initial conf
	ros::Subscriber joint_states_sub = nh.subscribe("joint_states", 1, readJointPos);
	joint_states_arrived = false;
	while (!joint_states_arrived)
	{
		ros::Duration(1.0).sleep();
		ros::spinOnce();
	}
	joint_states_sub.shutdown();
	/* Trajectory initialization */
	jointCommand.position = q_read;
	jointCommand.velocity.resize(NUM_JOINTS);
	for (int i = 0; i < NUM_JOINTS; i++)
	{
		jointCommand.velocity[i] = 0.0;
	}
	jointTraj.joint_names = default_joint_names;

	/* Control loop */
	double t0 = ros::Time::now().toSec();
	double t = t0;
	while (ros::ok())
	{

		t = ros::Time::now().toSec() - t0;

		jointTrajPoint.positions = jointCommand.position;
		jointTrajPoint.velocities = jointCommand.velocity;
		jointTrajPoint.time_from_start = ros::Duration(t);

		/* Publish message */
		jointTraj.points.at(0) = jointTrajPoint;
		joint_cmd_pub.publish(jointTraj);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return (0);
}

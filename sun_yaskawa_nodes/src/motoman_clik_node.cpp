#include "ros/ros.h"
#include "sun_robot_lib/Robots/MotomanSIA5F.h"
#include "sun_robot_ros/clikNode.h"

/*GLOBAL ROS VARS*/
ros::Publisher pub_joints_cmd;
std::string joint_state_topic_str;
/*END Global ROS Vars*/

TooN::Vector<7> qR;
bool b_joint_state_arrived = false;
void joint_position_cb(const sensor_msgs::JointStateConstPtr &joi_state_msg)
{

    qR[0] = joi_state_msg->position[0];
    qR[1] = joi_state_msg->position[1];
    qR[2] = joi_state_msg->position[2];
    qR[3] = joi_state_msg->position[3];
    qR[4] = joi_state_msg->position[4];
    qR[5] = joi_state_msg->position[5];
    qR[6] = joi_state_msg->position[6];
    b_joint_state_arrived = true;
}

TooN::Vector<> get_joint_position_fcn()
{
    //wait joint position
    ros::NodeHandle nh_public;
    ros::Subscriber joint_position_sub = nh_public.subscribe(joint_state_topic_str, 1, joint_position_cb);
    b_joint_state_arrived = false;
    while (ros::ok() && !b_joint_state_arrived)
    {
        ros::spinOnce();
    }
    return qR;
}

void joint_publish_fcn(const TooN::Vector<> &qR, const TooN::Vector<> &dqR)
{

    sensor_msgs::JointState out_msg;
    out_msg.position.resize(7);
    out_msg.velocity.resize(7);

    out_msg.position[0] = qR[0];
    out_msg.position[1] = qR[1];
    out_msg.position[2] = qR[2];
    out_msg.position[3] = qR[3];
    out_msg.position[4] = qR[4];
    out_msg.position[5] = qR[5];
    out_msg.position[6] = qR[6];

    out_msg.velocity[0] = dqR[0];
    out_msg.velocity[1] = dqR[1];
    out_msg.velocity[2] = dqR[2];
    out_msg.velocity[3] = dqR[3];
    out_msg.velocity[4] = dqR[4];
    out_msg.velocity[5] = dqR[5];
    out_msg.velocity[6] = dqR[6];

    out_msg.header.frame_id = "yaskawa";
    out_msg.header.stamp = ros::Time::now();

    pub_joints_cmd.publish(out_msg);
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "motoman_clik");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public;

    //params
    nh_private.param("joint_state_topic", joint_state_topic_str, std::string("/motoman/joint_states"));
    std::string joint_command_topic_str;
    nh_private.param("joint_command_topic", joint_command_topic_str, std::string("/motoman/joint_ll_control"));

    //Subscribers

    //Publishers
    pub_joints_cmd = nh_public.advertise<sensor_msgs::JointState>(joint_command_topic_str, 1);

    sun::MotomanSIA5F sia5f("SIA5F");
    sun::clikNode clik_node(
        sia5f,
        get_joint_position_fcn,
        joint_publish_fcn,
        nh_public,
        nh_private);

    clik_node.run();
}
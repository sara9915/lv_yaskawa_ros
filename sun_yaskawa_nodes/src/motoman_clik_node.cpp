#include "ros/ros.h"
#include "sun_robot_lib/Robots/LBRiiwa7.h"
#include "sun_robot_ros/ClikNode.h"

namespace sun {
class motoman_ClikNode : public ClikNode {
private:
protected:
  ros::Publisher pub_joints_cmd_;
  ros::Subscriber joint_position_sub_;

public:
  motoman_ClikNode(
      const ros::NodeHandle &nh_for_topics = ros::NodeHandle("clik"),
      const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"))
      : ClikNode(std::make_shared<LBRiiwa7>("iiwa7"), nh_for_topics,
                 nh_for_parmas) {

    std::string joint_state_topic_str;
    nh_for_parmas.param("joint_state_topic", joint_state_topic_str,
                        std::string("/motoman/joint_states"));
    std::string joint_command_topic_str;
    nh_for_parmas.param("joint_command_topic", joint_command_topic_str,
                        std::string("/motoman/joint_ll_control"));

    // Subscribers
    joint_position_sub_ = nh_.subscribe(
        joint_state_topic_str, 1, &motoman_ClikNode::joint_position_cb, this);

    // Publishers
    pub_joints_cmd_ =
        nh_.advertise<sensor_msgs::JointState>(joint_command_topic_str, 1);
  }

  TooN::Vector<7> qR;
  bool b_joint_state_arrived = false;
  void joint_position_cb(const sensor_msgs::JointStateConstPtr &joi_state_msg) {

    qR[0] = joi_state_msg->position[0];
    qR[1] = joi_state_msg->position[1];
    qR[2] = joi_state_msg->position[2];
    qR[3] = joi_state_msg->position[3];
    qR[4] = joi_state_msg->position[4];
    qR[5] = joi_state_msg->position[5];
    qR[6] = joi_state_msg->position[6];
    b_joint_state_arrived = true;
  }

  //! Cbs
  virtual TooN::Vector<> getJointPositionRobot(bool wait_new_sample) override {
    // wait joint position
    if (wait_new_sample) {
      b_joint_state_arrived = false;
    }
    while (ros::ok() && !b_joint_state_arrived) {
      spinOnce();
    }
    return qR;
  }
  virtual void publishJointRobot(const TooN::Vector<> &qR,
                                 const TooN::Vector<> &qR_dot) override {

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

    out_msg.velocity[0] = qR_dot[0];
    out_msg.velocity[1] = qR_dot[1];
    out_msg.velocity[2] = qR_dot[2];
    out_msg.velocity[3] = qR_dot[3];
    out_msg.velocity[4] = qR_dot[4];
    out_msg.velocity[5] = qR_dot[5];
    out_msg.velocity[6] = qR_dot[6];

    out_msg.header.frame_id = "yaskawa";
    out_msg.header.stamp = ros::Time::now();

    pub_joints_cmd_.publish(out_msg);
  }
};
} // namespace sun

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "motoman_clik");

  ros::NodeHandle nh_private = ros::NodeHandle("~");
  ros::NodeHandle nh_public;

  sun::motoman_ClikNode clik_node(nh_public, nh_private);

  clik_node.run();
}
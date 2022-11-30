#ifndef SUN_YASKAWA_NODES_FkineNode_H_
#define SUN_YASKAWA_NODES_FkineNode_H_

#include "sensor_msgs/JointState.h"
#include "sun_robot_lib/Robots/MotomanSIA5F.h"
#include "sun_robot_ros/FkineNode.h"

namespace sun {

class Yaskawa_FkineNode : public FkineNode {
private:
  ros::Subscriber joint_sub_;

public:
  static std::shared_ptr<sun::MotomanSIA5F> makeMotoman() {
    return std::make_shared<sun::MotomanSIA5F>("SIA5F");
  }

  Yaskawa_FkineNode(
      const ros::NodeHandle &nh_for_topics = ros::NodeHandle("clik"),
      const ros::NodeHandle &nh_for_parmas = ros::NodeHandle("~"),
      ros::CallbackQueue *callbk_queue = ros::getGlobalCallbackQueue())
      : FkineNode(Yaskawa_FkineNode::makeMotoman(), nh_for_topics,
                  nh_for_parmas, callbk_queue) {}

  ~Yaskawa_FkineNode() = default;

  /**
     joint_sub_ = ...
   */
  virtual void registerJointSubscriber() override {
    joint_sub_ = nh_.subscribe("/motoman/joint_states", 1,
                               &Yaskawa_FkineNode::joint_position_cb, this);
  }

  void joint_position_cb(const sensor_msgs::JointStateConstPtr &joi_state_msg) {
    TooN::Vector<7> qR;
    qR[0] = joi_state_msg->position[0];
    qR[1] = joi_state_msg->position[1];
    qR[2] = joi_state_msg->position[2];
    qR[3] = joi_state_msg->position[3];
    qR[4] = joi_state_msg->position[4];
    qR[5] = joi_state_msg->position[5];
    qR[6] = joi_state_msg->position[6];

    updateJoint(qR);
    // updateJointVel(qdotR);
    
    publishAll();

  }
};
} // namespace sun

#endif
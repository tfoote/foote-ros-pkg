#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "intial_state_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(10);

  const double degree = M_PI/180;

  // robot state
  double angle=0;

  // message declarations
  geometry_msgs::TransformStamped odom_trans;
  sensor_msgs::JointState joint_state;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "nxt_brick";

  while (ros::ok()) {
    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(1);
    joint_state.position.resize(1);
    joint_state.name[0] ="right_servo_joint";
    joint_state.position[0] = angle;


    // update transform
    // (moving in a circle with radius=2)
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = cos(angle)*2;
    odom_trans.transform.translation.y = sin(angle)*2;
    odom_trans.transform.translation.z = 0.1;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

    //send the joint state and transform
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);

    angle += degree/0.4;

    // This will adjust as needed per iteration
    loop_rate.sleep();
  }


  return 0;
}

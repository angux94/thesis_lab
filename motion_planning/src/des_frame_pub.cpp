#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;
int main(int argc, char** argv){
  ros::init(argc, argv, "des_frame_pub");

  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<std_msgs::String>("/des_frame", 1000);

  ros::Rate rate(10.0);

  string des_frame;
  std_msgs::String msg;
  while (node.ok()){

    cout << "Enter the desired frame: " << endl;
    getline(cin, des_frame);
    cout << " " << endl;
    msg.data = des_frame;
    pub.publish(msg);
    rate.sleep();
  }

  ros::shutdown();
  return 0;
};

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

class Node {
public:
    Node(ros::NodeHandle& nh, ros::NodeHandle& mnh) : _meas(-1) {
        _measurement_sub = mnh.subscribe("measurement", 5, &Node::measurement_cb, this);
        _action_sub = nh.subscribe("action", 5, &Node::action_cb, this);
        _neutral_sub = nh.subscribe("neutral", 5, &Node::neutral_cb, this);
    }

    void measurement_cb(const std_msgs::Int32::ConstPtr& msg) {
        _meas = msg->data;
        ROS_INFO_STREAM("Received measurement: " << _meas);
    }

    void action_cb(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO_STREAM("Received action: " << msg->data);
        ros::Time end = ros::Time::now() + ros::Duration(5.0);
        while (ros::Time::now() < end) {
            ROS_INFO_STREAM("Measurement: " << _meas);
            ros::Duration(0.25).sleep();
        }
    }

    void neutral_cb(const std_msgs::Int32::ConstPtr& msg) {
        ROS_INFO_STREAM("Received measurement: " << msg->data);
    }

private:
    boost::shared_ptr<ros::AsyncSpinner> _measurement_spinner;
    ros::Subscriber _measurement_sub;
    ros::Subscriber _action_sub;
    ros::Subscriber _neutral_sub;
    int _meas;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wait_for_msg");
    ros::NodeHandle nh;
    ros::NodeHandle mnh;
    ros::CallbackQueue queue;

    mnh.setCallbackQueue(&queue);

    ros::AsyncSpinner measurement_spinner(1, &queue);
    measurement_spinner.start();

    Node wm(nh, mnh);
    ros::spin();

    return 0;
}
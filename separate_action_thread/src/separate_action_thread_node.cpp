#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

class Node {
public:
    Node(ros::NodeHandle& nh, ros::NodeHandle& mnh) : _meas(-1) {
        _measurement_sub = nh.subscribe("measurement", 5, &Node::measurement_cb, this);
        _action_sub = mnh.subscribe("action", 1, &Node::action_cb, this);
        _neutral_sub = nh.subscribe("neutral", 5, &Node::neutral_cb, this);
    }

    long get_measurement() {
        boost::mutex::scoped_lock lock(_mutex);
        return _meas;
    }

    void set_measurement(long meas) {
        boost::mutex::scoped_lock lock(_mutex);
        _meas = meas;
    }

    void measurement_cb(const std_msgs::Int64::ConstPtr& msg) {
        set_measurement(msg->data);
        ROS_INFO_STREAM("Received measurement: " << msg->data);
    }

    void action_cb(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO_STREAM("Received action: " << msg->data);
        ros::Time end = ros::Time::now() + ros::Duration(5.0);
        while (ros::Time::now() < end) {
            ROS_INFO_STREAM("Measurement: " << get_measurement());
            ros::Duration(0.25).sleep();
        }
    }

    void neutral_cb(const std_msgs::Int64::ConstPtr& msg) {
        ROS_INFO_STREAM("Received measurement: " << msg->data);
    }

private:
    boost::shared_ptr<ros::AsyncSpinner> _measurement_spinner;
    ros::Subscriber _measurement_sub;
    ros::Subscriber _action_sub;
    ros::Subscriber _neutral_sub;
    long _meas;
    boost::mutex _mutex;
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
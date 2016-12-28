#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

class WaitForMessage {
public:
    WaitForMessage(ros::NodeHandle& nh) : _nh(nh), _meas(-1), _running(false) {
        _measurement_sub = nh.subscribe("measurement", 5, &WaitForMessage::measurement_cb, this);
        _action_sub = nh.subscribe("action", 5, &WaitForMessage::action_cb, this);
        _neutral_sub = nh.subscribe("neutral", 5, &WaitForMessage::neutral_cb, this);
    }

    void measurement_cb(const std_msgs::Int32::ConstPtr& msg) {
        _meas = msg->data;
        ROS_INFO_STREAM("Received measurement: " << _meas);
    }

    void action_cb(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO_STREAM("Received action: " << msg->data);
        if (_running) {
            ROS_INFO("Already running");
            return;
        }
        _running = true;
        std_msgs::Int32::ConstPtr msg1 =
                ros::topic::waitForMessage<std_msgs::Int32>("measurement", _nh, ros::Duration(5));
        ros::spinOnce();
        ROS_INFO_STREAM("Received measurement from action: "
                        << msg->data << " " << (msg1 ? msg1->data : 0));

        msg1 = ros::topic::waitForMessage<std_msgs::Int32>("measurement2", _nh, ros::Duration(5));
        ROS_INFO_STREAM("Received another one from action: "
                        << msg->data << " " << (msg1 ? msg1->data : 0));
        _running = false;
    }

    void neutral_cb(const std_msgs::Int32::ConstPtr& msg) {
        ROS_INFO_STREAM("Received measurement: " << msg->data);
    }

private:
    ros::Subscriber _measurement_sub;
    ros::Subscriber _action_sub;
    ros::Subscriber _neutral_sub;
    ros::NodeHandle& _nh;
    int _meas;
    bool _running;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wait_for_msg");
    ros::NodeHandle nh;

    WaitForMessage wm(nh);
    ros::spin();

    return 0;
}
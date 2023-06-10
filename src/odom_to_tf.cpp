#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/create_timer_ros.h>

using std::placeholders::_1;

class OdomToTF : public rclcpp::Node {
    public:
        OdomToTF() : Node("odom_to_tf") {

              // Timer Callback
            publishing_rate = 100;
            if (publishing_rate <= 0)
            {
                std::stringstream ss;
                ss << "Invalid timer_rate of " << std::to_string(publishing_rate) << " using 1.0 instead!";
                RCLCPP_WARN(this->get_logger(), ss.str().c_str());
                publishing_rate = 1.0;
            }

            odom_received=false;

            std::string odom_topic;
            frame_id = this->declare_parameter("frame_id", std::string(""));
            child_frame_id = this->declare_parameter("child_frame_id", std::string(""));
            odom_topic = this->declare_parameter("odom_topic", std::string("/odom/perfect"));
            RCLCPP_INFO(this->get_logger(), "odom_topic set to %s", odom_topic.c_str());
            if (frame_id != "") {
                RCLCPP_INFO(this->get_logger(), "frame_id set to %s", frame_id.c_str());
            }
            else {
                RCLCPP_INFO(this->get_logger(), "frame_id was not set. The frame_id of the odom message will be used.");
            }
            if (child_frame_id != "") {
                RCLCPP_INFO(this->get_logger(), "child_frame_id set to %s", child_frame_id.c_str());
            }
            else {
                RCLCPP_INFO(this->get_logger(), "child_frame_id was not set. The child_frame_id of the odom message will be used.");
            }
            sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(), std::bind(&OdomToTF::odomCallback, this, _1));
            tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
              //Timer
            m_timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / publishing_rate),
                                    std::bind(&OdomToTF::timerCallback, this));

            std::cerr<<"odom to tf initilized"<<std::endl;
        }
    private:
        bool odom_received;
        double publishing_rate;
        rclcpp::TimerBase::SharedPtr m_timer;
        std::string frame_id, child_frame_id;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        nav_msgs::msg::Odometry::SharedPtr m_odom_msg;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            m_odom_msg=msg;
            //std::cerr<< "odom received" <<std::endl;
            odom_received=true;
        }
        
          // Timer Callback
        void timerCallback()
        {
             //std::cerr<< "timer callback" <<std::endl;
             
            if (odom_received==true){
                //std::cerr<< "time" <<this->get_clock()->now().nanoseconds()<<std::endl;
                nav_msgs::msg::Odometry::SharedPtr msg=m_odom_msg;
                geometry_msgs::msg::TransformStamped tfs_;
                tfs_.header = msg->header;
                tfs_.header.stamp = this->get_clock()->now();
                tfs_.header.frame_id = frame_id != "" ? frame_id : tfs_.header.frame_id;
                tfs_.child_frame_id = child_frame_id != "" ? child_frame_id : msg->child_frame_id;
                tfs_.transform.translation.x = msg->pose.pose.position.x;
                tfs_.transform.translation.y = msg->pose.pose.position.y;
                tfs_.transform.translation.z = msg->pose.pose.position.z;

                tfs_.transform.rotation = msg->pose.pose.orientation;

                tfb_->sendTransform(tfs_);
            }else{
                std::cerr<<"no odom recived"<<std::endl;
            }
    

        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTF>());
    rclcpp::shutdown();
    return 0;
}

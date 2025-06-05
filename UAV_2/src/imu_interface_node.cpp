#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class IMUInterfaceNode : public rclcpp::Node
{
public:
    IMUInterfaceNode() : Node("imu_interface_node")
    {
        // Declare parameters
        this->declare_parameter("input_topic", "/imu/data");
        this->declare_parameter("output_topic", "/imu/filtered");
        this->declare_parameter("frame_id", "imu_link");
        this->declare_parameter("publish_rate", 100.0);

        // Get parameters
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        // Create subscriber for IMU data from Gazebo
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_topic, 10,
            std::bind(&IMUInterfaceNode::imu_callback, this, std::placeholders::_1));

        // Create publishers
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic, 10);
        euler_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/euler", 10);

        // Create timer for periodic publishing of processed data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&IMUInterfaceNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "IMU Interface Node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Store the latest IMU data
        latest_imu_msg_ = *msg;
        data_received_ = true;

        // Process and republish immediately
        process_and_publish_imu();

        // Convert quaternion to Euler angles and publish
        publish_euler_angles();

        // Log IMU data (optional, can be commented out for performance)
        if (log_counter_++ % 100 == 0) { // Log every 100th message
            RCLCPP_INFO(this->get_logger(), 
                "IMU Data - Linear Acc: [%.3f, %.3f, %.3f], Angular Vel: [%.3f, %.3f, %.3f]",
                msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
                msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        }
    }

    void process_and_publish_imu()
    {
        if (!data_received_) return;

        // Create processed IMU message
        sensor_msgs::msg::Imu processed_imu = latest_imu_msg_;
        
        // Update timestamp and frame
        processed_imu.header.stamp = this->now();
        processed_imu.header.frame_id = frame_id_;

        // Apply any filtering or processing here
        // For example, simple low-pass filter on acceleration
        static bool first_run = true;
        static geometry_msgs::msg::Vector3 filtered_accel;
        
        if (first_run) {
            filtered_accel = processed_imu.linear_acceleration;
            first_run = false;
        } else {
            const double alpha = 0.1; // Filter coefficient
            filtered_accel.x = alpha * processed_imu.linear_acceleration.x + (1 - alpha) * filtered_accel.x;
            filtered_accel.y = alpha * processed_imu.linear_acceleration.y + (1 - alpha) * filtered_accel.y;
            filtered_accel.z = alpha * processed_imu.linear_acceleration.z + (1 - alpha) * filtered_accel.z;
        }

        // Use filtered acceleration
        processed_imu.linear_acceleration = filtered_accel;

        // Set covariance matrices (adjust based on your IMU specifications)
        for (int i = 0; i < 9; ++i) {
            processed_imu.orientation_covariance[i] = 0.0;
            processed_imu.angular_velocity_covariance[i] = 0.0;
            processed_imu.linear_acceleration_covariance[i] = 0.0;
        }
        
        // Set diagonal elements (variance values - adjust based on your IMU)
        processed_imu.orientation_covariance[0] = 0.01;     // roll variance
        processed_imu.orientation_covariance[4] = 0.01;     // pitch variance
        processed_imu.orientation_covariance[8] = 0.01;     // yaw variance
        
        processed_imu.angular_velocity_covariance[0] = 0.001;  // x angular velocity variance
        processed_imu.angular_velocity_covariance[4] = 0.001;  // y angular velocity variance
        processed_imu.angular_velocity_covariance[8] = 0.001;  // z angular velocity variance
        
        processed_imu.linear_acceleration_covariance[0] = 0.01;  // x acceleration variance
        processed_imu.linear_acceleration_covariance[4] = 0.01;  // y acceleration variance
        processed_imu.linear_acceleration_covariance[8] = 0.01;  // z acceleration variance

        // Publish processed IMU data
        imu_publisher_->publish(processed_imu);
    }

    void publish_euler_angles()
    {
        if (!data_received_) return;

        // Convert quaternion to Euler angles
        tf2::Quaternion q(
            latest_imu_msg_.orientation.x,
            latest_imu_msg_.orientation.y,
            latest_imu_msg_.orientation.z,
            latest_imu_msg_.orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Create and publish Euler angles message
        geometry_msgs::msg::Vector3Stamped euler_msg;
        euler_msg.header.stamp = this->now();
        euler_msg.header.frame_id = frame_id_;
        euler_msg.vector.x = roll;
        euler_msg.vector.y = pitch;
        euler_msg.vector.z = yaw;

        euler_publisher_->publish(euler_msg);
    }

    void timer_callback()
    {
        if (data_received_ && (this->now() - rclcpp::Time(latest_imu_msg_.header.stamp)).seconds() > 1.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No IMU data received for more than 1 second. Check Gazebo simulation.");
        }
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    sensor_msgs::msg::Imu latest_imu_msg_;
    std::string frame_id_;
    bool data_received_ = false;
    int log_counter_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUInterfaceNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting IMU Interface Node...");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in IMU Interface Node: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <algorithm>
#include <vector>
#include <string>
#include <rclcpp/qos.hpp>
#include <cmath> // For std::abs
#include <mutex>
#include <iostream>
#include <iterator>
#include <array>

class GazeboToRealRobotBridge : public rclcpp::Node
{
public:
    GazeboToRealRobotBridge()
        : Node("gazebo_to_real_robot_bridge")
    {
        target_positions_.resize(7, 0.0);
        previous_sim_positions_.resize(7, 0.0);
        current_sim_positions_.resize(7, 0.0);
        real_arm_joint_positions_.resize(7, 0.0);
        // Subscribe to the Gazebo joint states
        gazebo_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/a200_0000/platform/joint_states", 10,
            std::bind(&GazeboToRealRobotBridge::gazebo_joint_states_callback, this, std::placeholders::_1));
        // Set the QoS profile for the publisher to match the expected settings
        rclcpp::QoS qos_profile = rclcpp::QoS(10)
                                    .reliable()               // Reliable QoS
                                    .durability_volatile();   // Volatile durability
        
        // Subscribe to the arm target positions topic
        target_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/a200_0000/arm_0_joint_trajectory_controller/joint_trajectory", 10,
            std::bind(&GazeboToRealRobotBridge::target_positions_callback, this, std::placeholders::_1));

        // Subscribe to the /joint_states topic
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&GazeboToRealRobotBridge::joint_state_callback, this, std::placeholders::_1));

        // Publisher for the physical robot's joint trajectory controller
        robot_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", qos_profile);

        // Define the arm joint names you're interested in
        sim_arm_joint_names_ = {
            "arm_0_joint_1",
            "arm_0_joint_2",
            "arm_0_joint_3",
            "arm_0_joint_4",
            "arm_0_joint_5",
            "arm_0_joint_6",
            "arm_0_joint_7"};
        real_arm_joint_names_ = {
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7"  
        };
    }

private:

    void target_positions_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_positions_mutex);

        // Set the target positions from the first point in the trajectory message
        if (!msg->points.empty() && msg->points[0].positions.size() == target_positions_.size())
        {
                    // Extract current joint positions from the simulation
            for (size_t i = 0; i < sim_arm_joint_names_.size(); ++i)
            {
                auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), sim_arm_joint_names_[i]);
                if (it != msg->joint_names.end())
                {
                    // Get the index of the arm joint in the message
                    auto index = std::distance(msg->joint_names.begin(), it);

                    // Store the corresponding position
                    target_positions_[i] = msg->points[0].positions[index];
                }
            }
            is_commanded_move_ = true; // Mark that a command has been issued
            RCLCPP_INFO(this->get_logger(), "Received new target positions.");
        }
    }
    // Method to reject simulation reported joint positions noise (this can be significant sometimes)
    bool is_position_change_significant(const std::vector<float>& new_positions, const std::vector<float>& last_positions, double deadband_threshold)
    {
        // Ensure both vectors are the same size
        if (new_positions.size() != last_positions.size()) {
            RCLCPP_ERROR(this->get_logger(), "Position vectors are not the same size.");
            return false;
        }

        max_sim_consec_position_difference = 0.0;

        for (size_t i = 0; i < new_positions.size(); ++i)
        {
            // Calculate the absolute difference between each corresponding position
            double difference = std::abs(new_positions[i] - last_positions[i]);

            // Track the maximum difference found
            if (difference > max_sim_consec_position_difference)
            {
                max_sim_consec_position_difference = difference;
            }
        }

        // Compare the maximum difference to the deadband threshold
        return max_sim_consec_position_difference > deadband_threshold;
    }



    // Callback function to handle the incoming joint states from Gazebo
    void gazebo_joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_positions_mutex); // Lock the mutex to prevent race conditions
        // Extract current joint positions from the simulation
        for (size_t i = 0; i < sim_arm_joint_names_.size(); ++i)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), sim_arm_joint_names_[i]);
            if (it != msg->name.end())
            {
                // Get the index of the arm joint in the message
                auto index = std::distance(msg->name.begin(), it);

                // Store the corresponding position
                current_sim_positions_[i] = msg->position[index];
            }
        }

        // Check if the robot is moving to a known configuration
        if (is_commanded_move_)
        {
            
            // Check if the current simulation positions match the target positions within the tolerance
            target_reached = false;
            max_sim_position_difference = 0.0;

            for (size_t i = 0; i < current_sim_positions_.size(); ++i)
            {
                // Check if the current simulation positions match the target positions within the tolerance
                double difference = std::abs(current_sim_positions_[i] - target_positions_[i]);
                if (difference > max_sim_position_difference) {
                    max_sim_position_difference = difference;  // Track the maximum difference
                }
            }

            if (max_sim_position_difference < tol)
            {
                target_reached = true;
                // If target is reached, disable commanded move flag
                is_commanded_move_ = false;
                RCLCPP_INFO(this->get_logger(), "Target position reached, re-enabling noise filter.");
            }


        }


        if(!init_reached){
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }
        
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = real_arm_joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint traj_point;

        // Filter only the arm joint positions from the incoming JointState message
        for (const auto &sim_arm_joint_name : sim_arm_joint_names_)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), sim_arm_joint_name);
            if (it != msg->name.end())
            {
                // Get the index of the arm joint in the message
                auto index = std::distance(msg->name.begin(), it);

                // Append the corresponding position to the trajectory point
                traj_point.positions.push_back(msg->position[index]);
            }
        }

        if(!init_reached){

            std::cout << "Joints positional commands" << std::endl;
            std::copy(traj_point.positions.begin(), traj_point.positions.end(), std::ostream_iterator<float>(std::cout, " "));
            std::cout << "" << std::endl;
            std::cout << "Real-life joints positons" << std::endl;
            std::for_each(std::begin(real_arm_joint_positions_), std::end(real_arm_joint_positions_), [](double val) { std::cout << val << " "; });
            std::cout << "" << std::endl;
            // Calculate the time from start based on the difference between real and simulated joint positions
            max_simtoreal_position_difference = 0.0;

            for (size_t i = 0; i < traj_point.positions.size(); ++i)
            {
                // Calculate the absolute difference between real and simulated joint positions
                double position_difference = std::abs(traj_point.positions[i] - real_arm_joint_positions_[i]);
                if (position_difference > max_simtoreal_position_difference) {
                    max_simtoreal_position_difference = position_difference;  // Track the maximum difference
                }
            }

            std::cout << "Max position difference: " << max_simtoreal_position_difference << std::endl;

            if (max_simtoreal_position_difference <= tol){
                std::cout << "Robot sync with the simulation initial position reached!" << std::endl;
                init_reached = true;
            }

            // Define a scaling factor for time (adjust this value as necessary)
            double scaling_factor = 3.0; // This controls how much time is given based on the max difference

            // Calculate time based on the difference
            double total_time = max_simtoreal_position_difference * scaling_factor;
            // Set time_from_start based on the maximum position difference
            traj_point.time_from_start.sec = static_cast<int>(total_time);
            traj_point.time_from_start.nanosec = static_cast<int>((total_time - traj_point.time_from_start.sec) * 1e9);


        }
        else{
            traj_point.time_from_start.sec = 0;
        }

        // Add the trajectory point to the message
        traj_msg.points.push_back(traj_point);

        // Only apply noise filtering if not in a commanded move
        if (!is_commanded_move_)
        {
            // Check if there is a significant change in simulation positions
            if (!is_position_change_significant(current_sim_positions_, previous_sim_positions_, deadband_threshold))
            {
                // If no significant change, return early without publishing
                std::cout << "No significant change in simulation joint positions. Skipping trajectory update." << std::endl;
                std::cout << "Previous simulation joints positons" << std::endl;
                std::for_each(std::begin(previous_sim_positions_), std::end(previous_sim_positions_), [](float val) { std::cout << val << " "; });
                std::cout << "" << std::endl;
                std::cout << "Current simulation joints positons" << std::endl;
                std::for_each(std::begin(current_sim_positions_), std::end(current_sim_positions_), [](float val) { std::cout << val << " "; });
                std::cout << "" << std::endl;
                return;
            }
        }
        // Update previous_sim_positions_ with the current positions since the change is significant
        previous_sim_positions_ = current_sim_positions_;

        // Publish the trajectory message to the physical robot
        robot_pub_->publish(traj_msg);
    }

    // Callback function to handle joint state messages coming from the rl robot
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_positions_mutex); // Lock the mutex to prevent race conditions
        
        // Ensure that real_arm_joint_positions_ has the correct size (already resized)
        real_arm_joint_positions_.clear();
        real_arm_joint_positions_.resize(real_arm_joint_names_.size(), 0.0);
        

        // Loop through the real_arm_joint_names_ and update the corresponding positions directly
        for (size_t i = 0; i < real_arm_joint_names_.size(); ++i)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), real_arm_joint_names_[i]);
            if (it != msg->name.end())
            {
                // Get the index of the arm joint in the message
                auto index = std::distance(msg->name.begin(), it);

                // Assign the corresponding position directly to the vector
                real_arm_joint_positions_[i] = msg->position[index];
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr target_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr robot_pub_;

    // List of arm joint names you're interested in
    std::vector<std::string> real_arm_joint_names_;
    std::vector<std::string> sim_arm_joint_names_;
    std::vector<float> previous_sim_positions_;
    std::vector<float> current_sim_positions_;
    // Vector to store the real-life robot's joints positions
    std::vector<double> real_arm_joint_positions_;
    std::vector<float> target_positions_; // The target positions to reach
    // Mutex to protect access to real_arm_joint_positions_
    std::mutex joint_positions_mutex;
    bool init_reached = false;
    double tol = 0.02;
    double deadband_threshold = 0.04; // Adjust this value based on your noise tolerance
    bool is_commanded_move_ = false; // If the robot is moving toward a known target
    bool target_reached = false;
    double max_simtoreal_position_difference;
    double max_sim_consec_position_difference;
    double max_sim_position_difference;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboToRealRobotBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

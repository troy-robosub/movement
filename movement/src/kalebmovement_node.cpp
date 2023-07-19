#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>
#include <numeric>
#include <vector>
#include <ros/time.h>

// Global variables
ros::Publisher vel_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

// Function to arm the vehicle
void arm() {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    arming_client.call(arm_cmd);
}

// Function to set the mode of the vehicle
void set_mode(std::string mode) {
    mavros_msgs::SetMode set_mode_cmd;
    set_mode_cmd.request.custom_mode = mode;
    set_mode_client.call(set_mode_cmd);
}

// Function to move the vehicle in the x direction
void travel_in_x(double xThrottle, double distanceTravel) {
    // Set the mode to STABILIZE
    set_mode("STABILIZE");

    // Start the timer
    ros::Time start_time = ros::Time::now();

    // Create an array to store velocity values
    std::vector<double> velocity_array;

    // Distance is initially 0
    double distance = 0;

    // Loop until the desired distance is reached
    while (distance < distanceTravel) {
        // Create a TwistStamped message to send velocity commands
        geometry_msgs::TwistStamped vel_msg;
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.twist.linear.x = xThrottle;
        vel_pub.publish(vel_msg);

        // Get the elapsed time
        ros::Duration elapsed_time = ros::Time::now() - start_time;

        // Get the velocity at each iteration
        double velocity = std::sqrt(std::pow(vel_msg.twist.linear.x, 2));

        // Add the velocity to the array
        velocity_array.push_back(velocity);

        // Get the average velocity
        double average_velocity = std::accumulate(velocity_array.begin(), velocity_array.end(), 0.0) / velocity_array.size();

        // Calculate the distance traveled
        distance = elapsed_time.toSec() * average_velocity;

        // Print the distance
        ROS_INFO("RECORDED DISTANCE: %f", distance);
    }

    // Stop the vehicle
    geometry_msgs::TwistStamped stop_msg;
    stop_msg.header.stamp = ros::Time::now();
    vel_pub.publish(stop_msg);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "kalebmovement_node");

    // Set the desired distance to travel
    double distanceTravel = 2.0;

    // Set the throttle for the x direction
    double xThrottle = 0.5;

    // Call the travel_in_x function to make the robot travel in the x direction
    travel_in_x(distanceTravel, xThrottle);

    return 0;
}

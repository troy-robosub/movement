#include <ros/ros.h>
#include <ros/time.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <cmath>
#include <numeric>
#include <vector>

// Global variable to store the current depth
double current_depth = 0.0;



// Global variable to store the velocity
geometry_msgs::Vector3 velocity; // its three components are accessed by velocity.x, velocity.y, velocity.z

// Global variable to store the heading
double heading = 0.0;

// Forward declarations so you can use functions in other functions
void manualControl(float x, float y, float z, float r, ros::NodeHandle& nh);
void arm(ros::NodeHandle& nh);
void depthCallback(const std_msgs::Float64::ConstPtr& msg);
double getDepth(ros::NodeHandle& nh);
void goDepth(ros::NodeHandle& nh, double depth);
void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void rotateClockwise(double degrees, ros::NodeHandle& nh);
void rotateCounterClockwise(double degrees, ros::NodeHandle& nh);
void maintainHeading(double heading, ros::NodeHandle& nh);


void manualControl(int ch1, int ch2, int ch3, int ch4, ros::NodeHandle& nh) {
    ros::Publisher rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

    mavros_msgs::OverrideRCIn rc_override_msg;
    rc_override_msg.channels[0] = ch1;
    rc_override_msg.channels[1] = ch2;
    rc_override_msg.channels[2] = ch3;
    rc_override_msg.channels[3] = ch4;
    for(int i=4; i<18; i++) {
        rc_override_msg.channels[i] = 0; // Set the rest of the channels to 0
    }

    rc_override_pub.publish(rc_override_msg);
}

// Function to arm the vehicle
void arm(ros::NodeHandle& nh) {
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming",1000);
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    arming_client.call(arm_cmd);
}

// Callback function for the depth subscriber
void depthCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // Convert the altitude to a depth
    current_depth = -msg->data;
}

double getDepth(ros::NodeHandle& nh)
{
    // Create a subscriber for the relative altitude
    ros::Subscriber sub = nh.subscribe("/mavros/global_position/rel_alt", 1, depthCallback);

    // Wait for a depth reading
    while (current_depth == 0.0 && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // Print the depth
    ROS_INFO("Current Depth: %f", current_depth);

    return current_depth;
}

void goDepth(ros::NodeHandle& nh, double depth)
{
    // Create a publisher for the velocity command
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel", 1);

    // Create a subscriber for the depth
    ros::Subscriber sub = nh.subscribe("/mavros/global_position/rel_alt", 1, depthCallback);

    // Create a velocity command message
    geometry_msgs::Twist cmd_vel;

    // Wait for a depth reading
    while (current_depth == 0.0 && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // If the current depth is greater than the desired depth
    if (current_depth > depth) {
        // Set thrust to upward
        cmd_vel.linear.z = 0.7;

        // Run this until the current depth is less than the desired depth
        while (current_depth > depth && ros::ok()) {
            // Publish the velocity command
            pub.publish(cmd_vel);

            // Spin and sleep
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }
    // If the current depth is less than the desired depth
    else {
        // Set thrust to downward
        cmd_vel.linear.z = -0.3;

        // Run this until the current depth is greater than the desired depth
        while (current_depth < depth && ros::ok()) {
            // Publish the velocity command
            pub.publish(cmd_vel);

            // Spin and sleep
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }

    // Stop the vehicle by setting the velocity to 0
    cmd_vel.linear.z = 0;
    pub.publish(cmd_vel);

    // Print the depth reached
    ROS_INFO("REACHED DESIRED Depth: %f", current_depth);
}

// Callback function for the velocity subscriber
void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    // Store the velocity
    velocity = msg->twist.linear;
}

double getVelocity(ros::NodeHandle& nh)
{
    // Create a subscriber for the velocity
    ros::Subscriber sub = nh.subscribe("/mavros/global_position/raw/vel", 1, velocityCallback);

    // Wait for a velocity reading
    while (velocity.x == 0.0 && velocity.y == 0.0 && velocity.z == 0.0 && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // Calculate the magnitude of the velocity
    double magnitude = sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z); // extended pythagorean theorem to get magnitude of 3D vector (velocity)

    // Print the velocity
    ROS_INFO("Current Velocity: %f", magnitude);

    return magnitude;
}

// Callback function for the heading subscriber
void headingCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // Store the heading
    heading = msg->data;
}

double getHeading(ros::NodeHandle& nh)
{
    // Create a subscriber for the heading
    ros::Subscriber sub = nh.subscribe("/mavros/global_position/compass_hdg", 1, headingCallback);

    // Wait for a heading reading
    while (heading == 0.0 && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // Print the heading
    ROS_INFO("Current Heading: %f", heading);

    return heading;
}

void maintainHeading(double heading, ros::NodeHandle& nh) {
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode", 1000);
    // Set mode to ALT_HOLD
    mavros_msgs::SetMode set_mode_msg;
    set_mode_msg.request.custom_mode = "ALT_HOLD";
    set_mode_client.call(set_mode_msg);

    // Get the current heading
    double start_heading = getHeading(nh);
    if (std::isnan(start_heading)) {
        ROS_WARN("No heading data received.");
        return;
    }

    // Calculate the difference in headings
    double angle = start_heading - heading;

    // If the angle is negative, rotate clockwise (increase heading)
    if (angle < 0) {
        rotateClockwise(abs(angle),nh);
    }
    // If the angle is positive, rotate counter-clockwise (decrease heading)
    else {
        rotateCounterClockwise(angle, nh);
    }

    // Set mode to ALT_HOLD again
    set_mode_msg.request.custom_mode = "ALT_HOLD";
    set_mode_client.call(set_mode_msg);
}

// Function to set the mode of the vehicle
void set_mode(std::string mode, ros::NodeHandle& nh) {
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode", 1000);
    mavros_msgs::SetMode set_mode_cmd;
    set_mode_cmd.request.custom_mode = mode;
    set_mode_client.call(set_mode_cmd);
}

// Function to move the vehicle in the x direction
void travel_in_x(double distance, double xThrottle, ros::NodeHandle& nh) {
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1000); // for travel_in_x()
    // Get the start time
    ros::Time start_time = ros::Time::now();

    // Create a vector to store the velocities
    std::vector<double> velocity_array;

    // Loop until the desired distance is reached
    double distance_traveled = 0;
    while (distance_traveled < distance) {
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
        distance_traveled = elapsed_time.toSec() * average_velocity;

        // Print the distance
        ROS_INFO("RECORDED DISTANCE: %f", distance_traveled);
    }

    // Stop the vehicle
    geometry_msgs::TwistStamped stop_msg;
    stop_msg.header.stamp = ros::Time::now();
    vel_pub.publish(stop_msg);
}

//experimental, adjust speed variable to change speed of vehicle which corresponds to 1600 input on channel 0 of rc_override_msg
void goForward(ros::NodeHandle& nh, double distance) {
    ros::Publisher rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    ros::Time start_time = ros::Time::now();
    
    // frequency rate
    ros::Rate rate(10); // 10 Hz
    
    double speed = 1.0; // Assume a constant speed of 1 m/s
    double distance_traveled = 0.0;

    while (distance_traveled < distance) {
        mavros_msgs::OverrideRCIn rc_override_msg;
        rc_override_msg.channels[0] = 1600; // Increase throttle on the first channel
        for(int i=1; i<18; i++) {
            rc_override_msg.channels[i] = 0; // Set the rest of the channels to 0
        }

        rc_override_pub.publish(rc_override_msg);

        ros::Duration elapsed_time = ros::Time::now() - start_time;
        distance_traveled = speed * elapsed_time.toSec();

        ros::spinOnce();
        rate.sleep();
    }

    // Stop the UUV by setting the throttle to 0
    mavros_msgs::OverrideRCIn rc_override_msg;
    rc_override_msg.channels[0] = 0;
    for(int i=1; i<18; i++) {
        rc_override_msg.channels[i] = 0;
    }

    rc_override_pub.publish(rc_override_msg);
}

void rotateClockwise(double degrees, ros::NodeHandle& nh) {
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode", 1000);
    // Set mode to ALT_HOLD
    mavros_msgs::SetMode set_mode_msg;
    set_mode_msg.request.custom_mode = "ALT_HOLD";
    set_mode_client.call(set_mode_msg);

    // Get the starting heading
    double start_heading = getHeading(nh);
    if (std::isnan(start_heading)) {
        ROS_WARN("No heading data received.");
        return;
    }

    // Initialize rotation
    double rotation = 0;

    // Rotate until the desired rotation is achieved
    for (int i = 0; i < 10000000; i++) {
        double current_heading = getHeading(nh);
        if (std::isnan(current_heading)) {
            ROS_WARN("No heading data received.");
            return;
        }
        rotation = abs(start_heading - current_heading);
        manualControl(0, 0, 500, 250, nh);
        if (rotation > 0.96 * degrees) {
            break;
        }
    }

    // Print the rotation reached
    ROS_INFO("ROTATED: %f", rotation);

    // Set mode to ALT_HOLD again
    set_mode_msg.request.custom_mode = "ALT_HOLD";
    set_mode_client.call(set_mode_msg);
}

void rotateCounterClockwise(double degrees, ros::NodeHandle& nh) {
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode", 1000);
    // Set mode to ALT_HOLD 
    mavros_msgs::SetMode set_mode_msg;
    set_mode_msg.request.custom_mode = "ALT_HOLD";
    set_mode_client.call(set_mode_msg);

    // Get the starting heading
    double start_heading = getHeading(nh);
    if (std::isnan(start_heading)) {
        ROS_WARN("No heading data received.");
        return;
    }

    // Initialize rotation
    double rotation = 0;

    // Rotate until the desired rotation is achieved
    for (int i = 0; i < 10000000; i++) {
        double current_heading = getHeading(nh);
        if (std::isnan(current_heading)) {
            ROS_WARN("No heading data received.");
            return;
        }
        rotation = abs(start_heading - current_heading);
        manualControl(0, 0, 500, -250, nh);
        if (rotation > 0.96 * degrees) {
            break;
        }
    }

    // Print the rotation reached
    ROS_INFO("ROTATED: %f", rotation);

    // Set mode to ALT_HOLD again
    set_mode_msg.request.custom_mode = "ALT_HOLD";
    set_mode_client.call(set_mode_msg);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "kalebmovement");
    // Create a ROS node handle
    ros::NodeHandle nh;

    // Set the desired distance to travel
    double distanceTravel = 10.0;

    // Set the throttle for the x direction
    double xThrottle = 0.5;

    arm(nh);

    set_mode("MANUAL", nh);
    rotateClockwise(90, nh);

    // Call the travel_in_x function to make the robot travel in the x direction
    // goForward(nh, distanceTravel);

    return 0;
}

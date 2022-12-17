#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"

#include <thread>
#include <string>
#include <mutex>

#include "ck_utilities/todd_trajectory/swerve_trajectory_smoother.hpp"
#include "ck_utilities/geometry/geometry_ros_helpers.hpp"
#include "ck_utilities/Units.hpp"

ros::NodeHandle* node;

using SwerveTrajectory::BasicTrajectory;
using SwerveTrajectory::BasicTrajectoryPoint;
using SwerveTrajectory::DetailedTrajectory;
using SwerveTrajectory::DetailedTrajectoryPoint;
using SwerveTrajectory::SwerveTrajectorySmoother;
using SwerveTrajectory::SwerveTrajectorySmootherConfiguration;

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "swerve_trajectory_follower_node");
	std::cout << "Starting" << std::endl;

	BasicTrajectory test_trajectory;

	BasicTrajectoryPoint first_point;
	first_point.speed = 0;
	test_trajectory.points.push_back(first_point);

	BasicTrajectoryPoint second_point;
	second_point.pose.position.x(10.0);
	second_point.speed = 1;
	test_trajectory.points.push_back(second_point);

	BasicTrajectoryPoint third_point;
	third_point.pose.position.x(10.0);
	third_point.pose.position.y(10.0);
	third_point.speed = 1;
	test_trajectory.points.push_back(third_point);

	SwerveTrajectorySmootherConfiguration config;
	config.heading_turn_rate_by_speed.insert(1, ck::math::deg2rad(2));
	config.track_acceleration_by_speed.insert(1, 1);
	config.track_deceleration_by_speed.insert(1, 1);
	config.track_turn_rate_by_speed.insert(1, ck::math::deg2rad(0.001));
	config.time_step_seconds = 0.1;

	SwerveTrajectorySmoother smoother(config);
	std::cout << "About to smooth" << std::endl;

	DetailedTrajectory result = smoother.smooth_path(test_trajectory);

	std::cout << "I survived." << std::endl;

	ros::NodeHandle n;

	node = &n;
	static ros::Publisher base_plan_publisher = node->advertise<geometry_msgs::PoseArray>("/InputPlan", 1);
	static ros::Publisher detailed_plan_publisher = node->advertise<geometry_msgs::PoseArray>("/DetailedPlan", 1);

	ros::Rate rate_limit(10);

	geometry_msgs::PoseArray base_plan;
	base_plan.header.frame_id = "map";
	geometry_msgs::PoseArray detailed_plan;
	detailed_plan.header.frame_id = "map";

	for (auto & base_point : test_trajectory.points)
	{
		base_plan.poses.push_back(geometry::to_msg(base_point.pose));
	}

	for (auto & detailed_point : result.points)
	{
		detailed_plan.poses.push_back(geometry::to_msg(detailed_point.pose));
	}

	while(!ros::isShuttingDown())
	{
		base_plan_publisher.publish(base_plan);
		detailed_plan_publisher.publish(detailed_plan);
		ros::spinOnce();
		rate_limit.sleep();
	}

	return 0;
}
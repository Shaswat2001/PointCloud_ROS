#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <iostream>
#include "math.h"
#include <sstream>
#include <string>
#include "yaml-cpp/yaml.h"
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;
const double tau = 2 * M_PI;

int main(int arg,char **argv)
{
    ros::init(arg,argv,"moveit_kuka");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    YAML::Node config = YAML::LoadFile("/home/shaswatg/UofM_ws/src/kuka_d435/config/way_point.yaml");
    YAML::Node char_tp=config["point_lt"];
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose ee_pose;
    float box_centre[]={0.5,0,0.1};
    double x,y,z;
    for(YAML::const_iterator it=char_tp.begin();it!=char_tp.end();++it)
    {
        std::string key = it->first.as<std::string>();
        x = box_centre[0] - char_tp[key]["x"].as<double>();
        y = box_centre[1] - char_tp[key]["y"].as<double>();
        z = box_centre[2] - char_tp[key]["z"].as<double>();
        target_pose.orientation.w = pow(1 + 2*x + pow(x,2) + pow(y,2),0.5)/2;
        target_pose.orientation.x = y*z/(4*target_pose.orientation.w);
        target_pose.orientation.y = (-x*z-z)/(4*target_pose.orientation.w);
        target_pose.orientation.z = 2*y/(4*target_pose.orientation.w);
        target_pose.orientation.w = cos(120);
        target_pose.position.x= char_tp[key]["x"].as<double>();
        target_pose.position.y= char_tp[key]["y"].as<double>();
        target_pose.position.z= char_tp[key]["z"].as<double>();
        waypoints.push_back(target_pose);
    }

    static const std::string planning_group="arm";
    moveit::planning_interface::MoveGroupInterface kuka_arm_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group = kuka_arm_group.getCurrentState() -> getJointModelGroup(planning_group);
    namespace rvit = rviz_visual_tools;

    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() =1.0;
    visual_tools.publishText(text_pose,"Welcome",rvit::WHITE,rvit::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Click Next : Display the Waypoints and Collision Objects");

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = kuka_arm_group.getPlanningFrame();
    collision_object.id = "box1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.1;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);
    kuka_arm_group.setMaxVelocityScalingFactor(0.2);
    kuka_arm_group.setPlanningTime(20.0);

    visual_tools.trigger();
    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = kuka_arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    my_plan.trajectory_ = trajectory;
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvit::WHITE, rvit::XLARGE);
    visual_tools.publishAxisLabeled(target_pose, "target");
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvit::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to execute the plan");


    kuka_arm_group.execute(my_plan);

    return 0;


}
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Function to add collision objects
void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 1;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = collision_objects[0].ADD;

    collision_objects[1].id = "object";
    collision_objects[1].header.frame_id = "base_link";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.02;
    collision_objects[1].primitives[0].dimensions[1] = 0.02;
    collision_objects[1].primitives[0].dimensions[2] = 0.2;
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 1;
    collision_objects[1].primitive_poses[0].position.y = 0;
    collision_objects[1].primitive_poses[0].position.z = 0.5;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

// Function to move to the home pose
void moveToHome(moveit::planning_interface::MoveGroupInterface& move_group) {
    std::map<std::string, double> home_joint_values;
    home_joint_values["elbow_joint"] = 2.094;
    home_joint_values["shoulder_lift_joint"] = -2.094;
    home_joint_values["shoulder_pan_joint"] = 0.0;
    home_joint_values["wrist_1_joint"] = -1.5708;
    home_joint_values["wrist_2_joint"] = -1.5708;
    home_joint_values["wrist_3_joint"] = 0.0;

    move_group.setJointValueTarget(home_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group.move();
        ROS_INFO("Moved to home position successfully!");
    } else {
        ROS_ERROR("Failed to move to home position!");
    }
}

/*
// Function to move to a specified pose
void moveToPose(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& target_pose) {
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group.move();
        ROS_INFO("Motion successful!");
    } else {
        ROS_ERROR("Motion failed!");
    }
}*/

// Function to move to the pre-pick pose with point-to-point motion
void moveToPrePickPose(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& pre_pick_pose) {
    move_group.setPoseTarget(pre_pick_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group.move();
        ROS_INFO("Pre-pick motion successful!");
    } else {
        ROS_ERROR("Pre-pick motion failed!");
    }
}

// Function to move to the pick pose with linear motion
void moveToPickPoseLinear(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& pick_pose) {
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(pick_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction == 1.0) {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;
        move_group.execute(my_plan);
        ROS_INFO("Linear pick motion successful!");
    } else {
        ROS_ERROR("Linear pick motion failed!");
    }
}

// Function to check for collisions
bool checkForCollisions(planning_scene::PlanningScene& planning_scene) {
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    //collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();

    // Get the current state of the robot
    robot_state::RobotState current_state = planning_scene.getCurrentStateNonConst();
    //robot_state::RobotState current_state = planning_scene.getCurrentState();

    planning_scene.checkCollision(collision_request, collision_result, current_state);

    ROS_INFO_STREAM("collision Test: "<< (collision_result.collision ? "in" : "not in") << " collision");
    return collision_result.collision;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur10_motion_planner");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);
    move_group.setPlanningTime(15.0);

    addCollisionObject(planning_scene_interface);
    ros::WallDuration(1.0).sleep();

    // Define prepick and pick poses
    tf2::Quaternion orientation;
    geometry_msgs::Pose prepick_pose;
    prepick_pose.position.x = 1;
    prepick_pose.position.y = 0;
    prepick_pose.position.z = 0.8;
    orientation.setRPY(M_PI, 0, 0);
    prepick_pose.orientation = tf2::toMsg(orientation);

    geometry_msgs::Pose pick_pose;
    pick_pose.position.x = 1;
    pick_pose.position.y = 0;
    pick_pose.position.z = 0.65;
    orientation.setRPY(M_PI, 0, 0);
    pick_pose.orientation = tf2::toMsg(orientation);
    
    // Check for collisions before moving to prepick pose
    if (checkForCollisions(planning_scene)) {
        ROS_ERROR("Collision detected before moving to prepick pose!");
    } else {
        moveToPrePickPose(move_group, prepick_pose);
    }
    
    // Check for collisions before moving to pick pose
    ROS_INFO("Checking for collisions before moving to pick pose.");
    if (checkForCollisions(planning_scene)) {
        ROS_ERROR("Collision detected before moving to pick pose!");
    } else {
        moveToPickPoseLinear(move_group, pick_pose);
    }

    ros::waitForShutdown();
    return 0;
}

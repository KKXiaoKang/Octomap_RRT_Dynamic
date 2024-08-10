#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fcl/fcl.h>

// 添加B样条曲线优化头文件
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathGeometric.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class RRTPlanner {
public:
    RRTPlanner() : nh_(), tf_listener_(tf_buffer_) {
        octomap_sub_ = nh_.subscribe("/octomap_full", 1, &RRTPlanner::octomapCallback, this);
        goal_sub_ = nh_.subscribe("/goal", 1, &RRTPlanner::goalCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("planned_path", 1);

        // Initialize OMPL
        space_ = ob::StateSpacePtr(new ob::SE3StateSpace());
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, -10);
        bounds.setHigh(0, 10);
        bounds.setLow(1, -10);
        bounds.setHigh(1, 10);
        bounds.setLow(2, 0);
        bounds.setHigh(2, 2);
        space_->as<ob::SE3StateSpace>()->setBounds(bounds);

        ss_ = std::make_shared<og::SimpleSetup>(space_);
        ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });

        // Set the planner
        auto planner = std::make_shared<og::RRTstar>(ss_->getSpaceInformation());
        ss_->setPlanner(planner);

        // Initialize goal_received_ flag
        goal_received_ = false;
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg) {
        ROS_INFO("Received Octomap, updating environment");
        octomap::AbstractOcTree* abstractTree = octomap_msgs::fullMsgToMap(*msg);
        if (!abstractTree) {
            ROS_ERROR("Error creating octree from received message");
            return;
        }

        octree_ = dynamic_cast<octomap::OcTree*>(abstractTree);
        if (!octree_) {
            ROS_ERROR("Received octree is not of type OcTree");
            delete abstractTree;
            return;
        }

        // Convert octomap to FCL collision object
        auto tree_ptr = std::make_shared<fcl::OcTreef>(std::shared_ptr<const octomap::OcTree>(octree_));
        octree_collision_object_ = std::make_shared<fcl::CollisionObjectf>(tree_ptr);

        if (goal_received_) {
            planPath();
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        ROS_INFO("Received new goal");
        goal_ = *msg;
        goal_received_ = true;

        // Print new goal position
        ROS_INFO_STREAM("New goal position: (" << goal_.pose.position.x << ", "
                                               << goal_.pose.position.y << ", "
                                               << goal_.pose.position.z << ")");

        // 清理之前的规划器状态
        ss_->clear();

        // Check if the goal is valid
        if (!octree_) {
            ROS_WARN("Octree not yet received, cannot plan path");
            return;
        }

        if (goal_received_) {
            planPath();
        }
    }

    bool isStateValid(const ob::State *state) const {
        const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
        fcl::Transform3f transform;
        transform.translation() = fcl::Vector3f(se3state->getX(), se3state->getY(), se3state->getZ());
        transform.linear() = fcl::Quaternionf(se3state->rotation().w, se3state->rotation().x, se3state->rotation().y, se3state->rotation().z).toRotationMatrix();

        fcl::CollisionRequestf request;
        fcl::CollisionResultf result;

        // Create a shared pointer for the robot's bounding box
        auto box = std::make_shared<fcl::Boxf>(0.6, 0.4, 1.2); // 根据机器人的实际尺寸调整 -- Robot bounding box size can be adjusted
        fcl::CollisionObjectf robot_collision_object(box, transform);

        fcl::collide(&robot_collision_object, octree_collision_object_.get(), request, result);
        // ROS_DEBUG_STREAM("Checking state validity for point: (" << se3state->getX() << ", " << se3state->getY() << ", " << se3state->getZ() << ") -> " << is_valid);
        return(!result.isCollision());
    }

    void planPath() {
        if (!octree_) {
            ROS_WARN("Octree not yet received, cannot plan path");
            return;
        }

        if (!goal_received_) {
            ROS_WARN("Goal not yet received, cannot plan path");
            return;
        }

        ob::ScopedState<> start(space_);
        ob::ScopedState<> goal(space_);

        start->as<ob::SE3StateSpace::StateType>()->setXYZ(0, 0, 0);  // Set start position
        start->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();
        goal->as<ob::SE3StateSpace::StateType>()->setXYZ(goal_.pose.position.x, goal_.pose.position.y, goal_.pose.position.z);   // Set goal position
        goal->as<ob::SE3StateSpace::StateType>()->rotation().setIdentity();

        // Check if start and goal states are valid
        if (!isStateValid(start.get())) {
            ROS_ERROR("Start state is invalid!");
            return;
        }

        if (!isStateValid(goal.get())) {
            ROS_ERROR("Goal state is invalid!");
            return;
        }

        ss_->setStartAndGoalStates(start, goal);
        ob::PlannerStatus solved = ss_->solve(1.0);

        if (solved) {
            ROS_INFO("Found solution!");
            og::PathGeometric path = ss_->getSolutionPath();
            path.interpolate();

            nav_msgs::Path path_msg;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = ros::Time::now();

            for (std::size_t i = 0; i < path.getStateCount(); ++i) {
                const auto *state = path.getState(i)->as<ob::SE3StateSpace::StateType>();
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.frame_id = "map";
                pose_stamped.header.stamp = ros::Time::now();
                pose_stamped.pose.position.x = state->getX();
                pose_stamped.pose.position.y = state->getY();
                pose_stamped.pose.position.z = state->getZ();
                pose_stamped.pose.orientation.w = state->rotation().w;
                pose_stamped.pose.orientation.x = state->rotation().x;
                pose_stamped.pose.orientation.y = state->rotation().y;
                pose_stamped.pose.orientation.z = state->rotation().z;
                path_msg.poses.push_back(pose_stamped);
            }
            path_pub_.publish(path_msg);
        } else {
            ROS_WARN("No solution found");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::shared_ptr<og::SimpleSetup> ss_;
    ob::StateSpacePtr space_;
    octomap::OcTree *octree_ = nullptr;
    geometry_msgs::PoseStamped goal_;
    bool goal_received_;

    std::shared_ptr<fcl::CollisionObjectf> octree_collision_object_; // 添加fcl碰撞对象
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rrt_planner");
    RRTPlanner planner;
    ros::spin();
    return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>

class OctomapProcessor {
public:
    OctomapProcessor() {
        sub_ = nh_.subscribe("/camera/depth/color/points", 1, &OctomapProcessor::pointCloudCallback, this);
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*cloud_msg, pcl_cloud);

        // 创建一个新的Octree
        octomap::OcTree tree(0.05); // 分辨率为0.05米

        // 将点云数据插入到Octree中
        for (auto& point : pcl_cloud.points) {
            tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
        }

        // 可选：进行Octree压缩
        tree.updateInnerOccupancy();

        // 将Octree转换为ROS消息并发布
        octomap_msgs::Octomap octomap_msg;
        octomap_msgs::binaryMapToMsg(tree, octomap_msg);
        octomap_msg.header.frame_id = cloud_msg->header.frame_id;
        octomap_msg.header.stamp = ros::Time::now();
        octomap_pub_.publish(octomap_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher octomap_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_processor_node");
    OctomapProcessor processor;
    ros::spin();
    return 0;
}

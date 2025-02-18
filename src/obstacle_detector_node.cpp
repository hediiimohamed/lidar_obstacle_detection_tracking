/* obstacle_detector_node.cpp

 * Copyright (C) 2021 SS47816

 * ROS2 Node for 3D LiDAR Obstacle Detection & Tracking Algorithms

**/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/detected_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "vision_msgs/msg/bounding_box3_d.hpp"
#include "vision_msgs/msg/bounding_box3_d_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include<Eigen/Dense>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "lidar_detection_tracking/box.hpp"
#include "lidar_detection_tracking/obstacle_detector.hpp"

namespace lidar_obstacle_detector {

    //Pointcloud Filtering Parametrs :
    bool USE_PCA_BOX;
    bool USE_TRACKING;
    float VOXEL_GRID_SIZE;
    Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;
    float GROUND_THRESH;
    float CLUSTER_THRESH;
    int CLUSTER_MAX_SIZE, CLUSTER_MIN_SIZE;
    float DISPLACEMENT_THRESH, IOU_THRESH;


class ObstacleDetectorNode : public rclcpp::Node
{
    public :
    ObstacleDetectorNode() : Node("obstacle_detector"), tf2_buffer_(this->get_clock()), tf2_listener_(tf2_buffer_)
    {   
        // Declare topic parameters
                this->declare_parameter<std::string>("lidar_points_topic", "/ouster/points");
                this->declare_parameter<std::string>("cloud_ground_topic", "/cloud/ground");
                this->declare_parameter<std::string>("cloud_clusters_topic", "/cloud/clusters");
                this->declare_parameter<std::string>("vision_bboxes_topic", "/bboxes/vision");
                this->declare_parameter<std::string>("autoware_objects_topic", "/detected_objects");
                this->declare_parameter<std::string>("bbox_target_frame", "base_link");
        // Declare parameters
                this->declare_parameter("use_pca_box", true);
                this->declare_parameter("use_tracking", true);
                this->declare_parameter("voxel_grid_size", 0.1f);
                this->declare_parameter("roi_max_x", 50.0f);
                this->declare_parameter("roi_max_y", 50.0f);
                this->declare_parameter("roi_max_z", 50.0f);
                this->declare_parameter("roi_min_x", -50.0f);
                this->declare_parameter("roi_min_y", -50.0f);
                this->declare_parameter("roi_min_z", -50.0f);
                this->declare_parameter("ground_threshold", 0.1f);
                this->declare_parameter("cluster_threshold", 0.5f);
                this->declare_parameter("cluster_max_size", 10000);
                this->declare_parameter("cluster_min_size", 100);
                this->declare_parameter("displacement_threshold", 1.0f);
                this->declare_parameter("iou_threshold", 0.5f);

        // Get parameters
            USE_PCA_BOX = this->get_parameter("use_pca_box").as_bool();
            USE_TRACKING = this->get_parameter("use_tracking").as_bool();
            VOXEL_GRID_SIZE = this->get_parameter("voxel_grid_size").as_double();

            ROI_MAX_POINT = Eigen::Vector4f(
                this->get_parameter("roi_max_x").as_double(),
                this->get_parameter("roi_max_y").as_double(),
                this->get_parameter("roi_max_z").as_double(),
                1.0f);

            ROI_MIN_POINT = Eigen::Vector4f(
                this->get_parameter("roi_min_x").as_double(),
                this->get_parameter("roi_min_y").as_double(),
                this->get_parameter("roi_min_z").as_double(),
                1.0f);

            GROUND_THRESH = this->get_parameter("ground_threshold").as_double();
            CLUSTER_THRESH = this->get_parameter("cluster_threshold").as_double();
            CLUSTER_MAX_SIZE = this->get_parameter("cluster_max_size").as_int();
            CLUSTER_MIN_SIZE = this->get_parameter("cluster_min_size").as_int();
            DISPLACEMENT_THRESH = this->get_parameter("displacement_threshold").as_double();
            IOU_THRESH = this->get_parameter("iou_threshold").as_double();

        // Get topic parameters

            lidar_points_topic_ = this->get_parameter("lidar_points_topic").as_string();
            cloud_ground_topic_ = this->get_parameter("cloud_ground_topic").as_string();
            cloud_clusters_topic_ = this->get_parameter("cloud_clusters_topic").as_string();
            vision_bboxes_topic_ = this->get_parameter("vision_bboxes_topic").as_string();
            autoware_objects_topic_ = this->get_parameter("autoware_objects_topic").as_string();
            bbox_target_frame_ = this->get_parameter("bbox_target_frame").as_string();
        //Vis
        // RCLCPP_INFO(this->get_logger(), "Lidar Points Topic: %s", lidar_points_topic_.c_str());
        // RCLCPP_INFO(this->get_logger(), "Cloud Ground Topic: %s", cloud_ground_topic_.c_str());
        // RCLCPP_INFO(this->get_logger(), "Cloud Clusters Topic: %s", cloud_clusters_topic_.c_str());
        // RCLCPP_INFO(this->get_logger(), "Vision Bboxes Topic: %s", vision_bboxes_topic_.c_str());
        // RCLCPP_INFO(this->get_logger(), "Autoware Objects Topic: %s", autoware_objects_topic_.c_str());
        // RCLCPP_INFO(this->get_logger(), "BBox Target Frame: %s", bbox_target_frame_.c_str());



        //susbcribers and publishers : 
            sub_lidar_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lidar_points_topic_, rclcpp::QoS(rclcpp::SystemDefaultsQoS()).best_effort(), std::bind(&ObstacleDetectorNode::lidarPointsCallback, this, std::placeholders::_1));

            pub_autoware_objects_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(autoware_objects_topic_, 10);
            pub_vision_bboxes_ = this->create_publisher<vision_msgs::msg::BoundingBox3DArray>(vision_bboxes_topic_, 10);
            pub_cloud_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_ground_topic_, 10);
            pub_cloud_clusters_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_clusters_topic_, 10);
            pub_bbox_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bbox_markers", 10);

        // Initialize Obstacle Detector
            obstacle_detector_ = std::make_shared<ObstacleDetector<pcl::PointXYZ>>();
            obstacle_id_ = 0;
    }
    private :

        // Variables
            size_t obstacle_id_;
            std::string bbox_target_frame_, bbox_source_frame;
            std::vector<Box> prev_boxes_, curr_boxes_;
            std::shared_ptr<ObstacleDetector<pcl::PointXYZ>> obstacle_detector_;

        // Topic names
            std::string lidar_points_topic_;
            std::string cloud_ground_topic_;
            std::string cloud_clusters_topic_;
            std::string vision_bboxes_topic_;
            std::string autoware_objects_topic_;

        //TF2 :
            tf2_ros::Buffer tf2_buffer_;
            tf2_ros::TransformListener tf2_listener_;

        //Subscribers :
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_points;

        //Publishers :
            rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr pub_autoware_objects_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray >::SharedPtr pub_bbox_markers;
            rclcpp::Publisher<vision_msgs::msg::BoundingBox3DArray>::SharedPtr pub_vision_bboxes_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_ground_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_clusters_;

//Extract Detected Objects and Ground Cloud from the Lidar PointCloud :
    void lidarPointsCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> lidar_points){

            RCLCPP_INFO(this->get_logger(), "Received Lidar PointCloud");
            //time the process :
                auto start_time = std::chrono::steady_clock::now();
            //Extract header and frame id :
                auto pointcloud_header = lidar_points->header;
                bbox_source_frame = pointcloud_header.frame_id;
            //Convert PointCloud2 to PCL PointCloud :
                pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*lidar_points, *raw_cloud);
            //Downsampling ROI :
                auto filtered_cloud = obstacle_detector_->filterCloud(raw_cloud, VOXEL_GRID_SIZE, ROI_MIN_POINT, ROI_MAX_POINT);
            //Segment Ground & obstacles :
                auto segmented_clouds = obstacle_detector_->segmentPlane(filtered_cloud, 30, GROUND_THRESH);
            //Cluster obstacles :
                auto cloud_clusters = obstacle_detector_->clustering(segmented_clouds.first, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);
            // Publish ground cloud & obstacle cloud
                publishClouds(std::move(segmented_clouds), pointcloud_header);

            // Publish Detected Objects
                publishDetectedObjects(std::move(cloud_clusters), pointcloud_header);
                
             //time the process :
                auto end_time = std::chrono::steady_clock::now();
                double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                RCLCPP_INFO(this->get_logger(), "The obstacle_detector_node found %d obstacles in %.3f second",
                    static_cast<int>(prev_boxes_.size()),
                    elapsed_time / 1000.0);
  }

//Publish Ground & Clustered Clouds Function
     void publishClouds(const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> &&segmented_clouds, const std_msgs::msg::Header &header)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing Ground & Clustered Clouds");

        sensor_msgs::msg::PointCloud2 cloud_ground;
        pcl::toROSMsg(*segmented_clouds.second, cloud_ground);
        cloud_ground.header = header;

        sensor_msgs::msg::PointCloud2 obstacle_cloud;
        pcl::toROSMsg(*segmented_clouds.first, obstacle_cloud);
        obstacle_cloud.header = header;

        pub_cloud_ground_->publish(cloud_ground);
        pub_cloud_clusters_->publish(obstacle_cloud);
    }

    //Vizualisation : Creating the MarkerArray to visualise the detected objects in Rviz
    visualization_msgs::msg::Marker transformMarker(const Box &box, const std_msgs::msg::Header &header, const geometry_msgs::msg::Pose &pose_transformed) { //const std_msgs::msg::Header &header,
        visualization_msgs::msg::Marker marker;
        // convert to Vision Marker :
        marker.header = header;
        marker.ns = "bounding_boxes";
        marker.id = box.id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose_transformed;
        marker.scale.x = box.dimension(0);
        marker.scale.y = box.dimension(1);
        marker.scale.z = box.dimension(2);
        marker.color.a = 0.3;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.7;

        return marker;

    }     

    //Creating the Bounding Boxes to visualise the detected objects Using either PCA For more precise and accurate orientation of the objects or Axis Aligned Bounding Boxes for simpler BB without orientation :

    void publishDetectedObjects(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &&cloud_clusters, const std_msgs::msg::Header &header) {
    RCLCPP_INFO(this->get_logger(), "Publishing Detected Objects");
    for (auto &cluster : cloud_clusters) {

        // Create bounding box :
        Box box = USE_PCA_BOX ? obstacle_detector_->pcaBoundingBox(cluster, obstacle_id_) : obstacle_detector_->axisAlignedBoundingBox(cluster, obstacle_id_);


        if (obstacle_id_ < __SIZE_MAX__) {
                        ++obstacle_id_;} 
        else { 
                        obstacle_id_ = 0;}
        curr_boxes_.emplace_back(box);
    }


    //Re-assign Box ids based on tracking results : Comparision of the curr_boxes_(that we got from the pcaBB or the AABB) and prev_boxes_
     if (USE_TRACKING) {obstacle_detector_->obstacleTracking(prev_boxes_, &curr_boxes_, DISPLACEMENT_THRESH, IOU_THRESH);
    }

    //Lookup for frame transform between Lidar frame and target frame :
    auto bbox_header = header;
    bbox_header.frame_id = bbox_target_frame_;
    geometry_msgs::msg::TransformStamped transform_stamped;

    try{

        transform_stamped = tf2_buffer_.lookupTransform(bbox_target_frame_, bbox_source_frame, tf2::TimePointZero);

    }
    catch(tf2::TransformException &ex){
        RCLCPP_WARN(this->get_logger(), "Transform Exception : %s", ex.what());
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform , outputtin obstacles in the original Lidar frame");
        bbox_header.frame_id = bbox_source_frame;
        try{
            transform_stamped = tf2_buffer_.lookupTransform(bbox_source_frame, bbox_source_frame, tf2::TimePointZero);
        }  
        catch(tf2::TransformException &ex){
            RCLCPP_ERROR(this->get_logger(), "Transform Exception : %s", ex.what());
            return;
        }

    }
    // Construct Bounding Boxes from the clusters
    visualization_msgs::msg::MarkerArray bbox_markers;
    for (auto &box : curr_boxes_) {
        geometry_msgs::msg::Pose pose, pose_transformed;
        pose.position.x = box.position(0);
        pose.position.y = box.position(1);
        pose.position.z = box.position(2);
        pose.orientation.w = box.quaternion.w();
        pose.orientation.x = box.quaternion.x();
        pose.orientation.y = box.quaternion.y();
        pose.orientation.z = box.quaternion.z();
        tf2::doTransform(pose, pose_transformed, transform_stamped);
        
        //convert to Autoware DetectedObject :

        bbox_markers.markers.emplace_back(transformMarker(box, bbox_header, pose_transformed));
    }
    pub_bbox_markers->publish(bbox_markers);

    //update previous boxes :
    prev_boxes_.swap(curr_boxes_);
    curr_boxes_.clear();
  };
};
} ; // namespace lidar_obstacle_detector

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_obstacle_detector::ObstacleDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
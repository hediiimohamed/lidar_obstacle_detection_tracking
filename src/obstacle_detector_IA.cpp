#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "vision_msgs/msg/bounding_box3_d.hpp"
#include "vision_msgs/msg/bounding_box3_d_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include<Eigen/Dense>
//#include "visualization_msgs/msg/marker_array.hpp"
//#include "visualization_msgs/msg/marker.hpp"
#include "lidar_detection_tracking/box.hpp"
#include "lidar_detection_tracking/obstacle_detector.hpp"


namespace lidar_obstacle_detector {

// Pointcloud Filtering Parameters
bool USE_PCA_BOX;
bool USE_TRACKING;
float VOXEL_GRID_SIZE;
Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;
float GROUND_THRESH;
float CLUSTER_THRESH;
int CLUSTER_MAX_SIZE, CLUSTER_MIN_SIZE;
float DISPLACEMENT_THRESH, IOU_THRESH;

class ObstacleDetectorNode : public rclcpp::Node {
 public:
  ObstacleDetectorNode() : Node("obstacle_detector_node"), tf2_buffer_(this->get_clock()), tf2_listener_(tf2_buffer_) {
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

    // Create subscribers and publishers
    sub_lidar_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ouster/points", 10, std::bind(&ObstacleDetectorNode::lidarPointsCallback, this, std::placeholders::_1));
    pub_cloud_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_ground_topic", 10);
    pub_cloud_clusters_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_clusters_topic", 10);
    pub_vision_bboxes_ = this->create_publisher<vision_msgs::msg::BoundingBox3DArray>("vision_bboxes_topic", 10);
    pub_autoware_objects_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjectsArray>("autoware_objects_topic", 10);

    // Create point processor
    obstacle_detector_ = std::make_shared<ObstacleDetector<pcl::PointXYZ>>();
    obstacle_id_ = 0;
  }

 private:
  size_t obstacle_id_;
  std::string bbox_target_frame_, bbox_source_frame_;
  std::vector<Box> prev_boxes_, curr_boxes_;
  std::shared_ptr<ObstacleDetector<pcl::PointXYZ>> obstacle_detector_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_clusters_;
  rclcpp::Publisher<vision_msgs::msg::BoundingBox3DArray>::SharedPtr pub_jsk_bboxes_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjectsArray>::SharedPtr pub_autoware_objects_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  void lidarPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_points) {
    RCLCPP_DEBUG(this->get_logger(), "lidar points received");
    const auto start_time = std::chrono::steady_clock::now();
    const auto pointcloud_header = lidar_points->header;
    bbox_source_frame_ = lidar_points->header.frame_id;

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*lidar_points, *raw_cloud);

    // Downsampling, ROI, and removing the car roof
    auto filtered_cloud = obstacle_detector_->filterCloud(
        raw_cloud, VOXEL_GRID_SIZE, ROI_MIN_POINT, ROI_MAX_POINT);

    // Segment the ground plane and obstacles
    auto segmented_clouds =
        obstacle_detector_->segmentPlane(filtered_cloud, 30, GROUND_THRESH);

    // Cluster objects
    auto cloud_clusters =
        obstacle_detector_->clustering(segmented_clouds.first, CLUSTER_THRESH,
                                      CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);

    // Publish ground cloud and obstacle cloud
    publishClouds(std::move(segmented_clouds), pointcloud_header);
    // Publish Obstacles
    publishDetectedObjects(std::move(cloud_clusters), pointcloud_header);

    // Time the whole process
    const auto end_time = std::chrono::steady_clock::now();
    const auto elapsed_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                              start_time);
    RCLCPP_INFO(this->get_logger(), "The obstacle_detector_node found %d obstacles in %.3f second",
                 static_cast<int>(prev_boxes_.size()),
                 static_cast<float>(elapsed_time.count() / 1000.0));
  }

  void publishClouds(const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> &&segmented_clouds,const std_msgs::msg::Header &header) {
    sensor_msgs::msg::PointCloud2::Ptr ground_cloud(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*(segmented_clouds.second), *ground_cloud);
    ground_cloud->header = header;

    sensor_msgs::msg::PointCloud2::Ptr obstacle_cloud(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*(segmented_clouds.first), *obstacle_cloud);
    obstacle_cloud->header = header;

    pub_cloud_ground_->publish(std::move(ground_cloud));
    pub_cloud_clusters_->publish(std::move(obstacle_cloud));
  }

  vision_msgs::msg::BoundingBox3D transformJskBbox(const Box &box, const std_msgs::msg::Header &header,const geometry_msgs::msg::Pose &pose_transformed) {
    vision_msgs::msg::BoundingBox3D jsk_bbox;
    jsk_bbox.header = header;
    jsk_bbox.center = pose_transformed;
    jsk_bbox.size.x = box.dimension(0);
    jsk_bbox.size.y = box.dimension(1);
    jsk_bbox.size.z = box.dimension(2);

    return std::move(jsk_bbox);
  }

  autoware_auto_perception_msgs::msg::DetectedObject transformAutowareObject(
      const Box &box, const std_msgs::msg::Header &header,
      const geometry_msgs::msg::Pose &pose_transformed) {
    autoware_auto_perception_msgs::msg::DetectedObject autoware_object;
    autoware_object.header = header;
    autoware_object.existence_probability = 1.0f;
    autoware_object.kinematics.pose_with_covariance.pose = pose_transformed;
    autoware_object.shape.dimensions.x = box.dimension(0);
    autoware_object.shape.dimensions.y = box.dimension(1);
    autoware_object.shape.dimensions.z = box.dimension(2);

    return std::move(autoware_object);
  }

  void publishDetectedObjects(
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &&cloud_clusters,
      const std_msgs::msg::Header &header) {
    for (auto &cluster : cloud_clusters) {
      // Create Bounding Boxes
      Box box =
          USE_PCA_BOX
              ? obstacle_detector_->pcaBoundingBox(cluster, obstacle_id_)
              : obstacle_detector_->axisAlignedBoundingBox(cluster, obstacle_id_);

      obstacle_id_ = (obstacle_id_ < SIZE_MAX) ? ++obstacle_id_ : 0;
      curr_boxes_.emplace_back(box);
    }

    // Re-assign Box ids based on tracking result
    if (USE_TRACKING)
      obstacle_detector_->obstacleTracking(prev_boxes_, &curr_boxes_,
                                          DISPLACEMENT_THRESH, IOU_THRESH);

    // Lookup for frame transform between the lidar frame and the target frame
    auto bbox_header = header;
    bbox_header.frame_id = bbox_target_frame_;
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf2_buffer_.lookupTransform(
          bbox_target_frame_, bbox_source_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      RCLCPP_WARN(
          this->get_logger(),
          "Frame Transform Given Up! Outputing obstacles in the original "
          "LiDAR frame %s instead...",
          bbox_source_frame_.c_str());
      bbox_header.frame_id = bbox_source_frame_;
      try {
        transform_stamped = tf2_buffer_.lookupTransform(
            bbox_source_frame_, bbox_source_frame_, tf2::TimePointZero);
      } catch (tf2::TransformException &ex2) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex2.what());
        return;
      }
    }

    // Construct Bounding Boxes from the clusters
    vision_msgs::msg::BoundingBox3DArray jsk_bboxes;
    jsk_bboxes.header = bbox_header;
    autoware_auto_perception_msgs::msg::DetectedObjectsArray autoware_objects;
    autoware_objects.header = bbox_header;

    // Transform boxes from lidar frame to base_link frame, and convert to jsk and
    // autoware msg formats
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

      jsk_bboxes.boxes.emplace_back(
          transformJskBbox(box, bbox_header, pose_transformed));
      autoware_objects.objects.emplace_back(
          transformAutowareObject(box, bbox_header, pose_transformed));
    }
    pub_jsk_bboxes_->publish(std::move(jsk_bboxes));
    pub_autoware_objects_->publish(std::move(autoware_objects));

    // Update previous bounding boxes
    prev_boxes_.swap(curr_boxes_);
    curr_boxes_.clear();
  }
};

}  // namespace lidar_obstacle_detector

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectorNode>();
  spin(node);  
  //auto node = std::make_shared<PCLPublisher>();
  rclcpp::shutdown();
  return 0;
}
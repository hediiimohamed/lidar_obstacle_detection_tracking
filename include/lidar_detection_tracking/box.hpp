#pragma once

#include <Eigen/Dense>
struct Box {
 public:
  int id;
  Eigen::Vector3f position;
  Eigen::Vector3f dimension;
  Eigen::Quaternionf quaternion;

  Box() {}

  Box(int id, Eigen::Vector3f position, Eigen::Vector3f dimension)
      : id(id),
        position(position),
        dimension(dimension),
        quaternion(Eigen::Quaternionf(1, 0, 0, 0)) {}

  Box(int id, Eigen::Vector3f position, Eigen::Vector3f dimension,
      Eigen::Quaternionf quaternion)
      : id(id),
        position(position),
        dimension(dimension),
        quaternion(quaternion) {}
};
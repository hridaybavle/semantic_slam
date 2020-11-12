#ifndef DETECTED_OBJECT_H
#define DETECTED_OBJECT_H

#include <fstream>
#include <functional>
#include <iostream>
#include <math.h>
#include <random>
#include <string>
#include <thread>

#include "Eigen/Dense"

struct detected_object {

  int id;
  float prob;
  float num_points;
  std::string type;
  std::string plane_type;
  Eigen::Vector3f pose;
  Eigen::Vector3f world_pose;
  Eigen::Vector4f normal_orientation;
};

#endif

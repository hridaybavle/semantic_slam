#ifndef DETECTED_OBJECT_H
#define DETECTED_OBJECT_H

#include <iostream>
#include <string>
#include <math.h>
#include <fstream>
#include <random>
#include <thread>
#include <functional>

#include "Eigen/Dense"


struct detected_object {

    int id;
    float prob;
    float num_points;
    std::string type;
    std::string plane_type;
    Eigen::Vector3f pose;
    Eigen::Vector4f normal_orientation;

};

#endif

#include <iostream>
#include <string>
#include <math.h>
#include <mutex>

#include "ros/ros.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "depth_synchronizer");
    ros::NodeHandle n;




    if(ros::ok())
    {
        //updating all the ros msgs
        ros::spin();

    }

    return 0;
}



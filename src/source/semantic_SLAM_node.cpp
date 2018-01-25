#include "semantic_SLAM.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "semantic_slam");
    ros::NodeHandle n;

    semantic_slam_ros mySemanticSLAM;
    mySemanticSLAM.open(n);

    ros::Rate r(30);

    while(ros::ok())
    {
        //updating all the ros msgs
        ros::spinOnce();
        //running the filter
        mySemanticSLAM.run();
        r.sleep();
    }

    return 0;
}



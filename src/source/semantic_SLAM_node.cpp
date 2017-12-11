#include "semantic_SLAM.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "semantic_slam");
    ros::NodeHandle n;

    semantic_SLAM mySemanticSLAM;
    mySemanticSLAM.open(n);

    ros::spin();

    return 0;
}



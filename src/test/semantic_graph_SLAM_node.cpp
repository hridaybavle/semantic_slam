#include "semantic_graph_slam.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "semantic_graph_slam");
    ros::NodeHandle n;

    semantic_graph_slam mySemanticGraphSLAM;
    mySemanticGraphSLAM.open(n);

    ros::Rate r(30);


 //   while(!mySemanticSLAM.pclViewer->wasStopped())
    {

        while(ros::ok())
        {
            //updating all the ros msgs
            ros::spinOnce();
            //running the filter
            mySemanticGraphSLAM.run();
            r.sleep();
        }

     //   mySemanticSLAM.pclViewer->spinOnce(100);
    }
    return 0;
}



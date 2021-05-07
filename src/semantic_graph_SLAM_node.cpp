#include "semantic_graph_slam_ros.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "semantic_graph_slam");
  ros::NodeHandle n;

  semantic_graph_slam_ros mySemanticGraphSLAM;
  mySemanticGraphSLAM.open(n);

  ros::Rate r(30);
  {

    while (ros::ok()) {
      // updating all the ros msgs
      ros::spinOnce();
      // running the filter
      mySemanticGraphSLAM.run();
      r.sleep();
    }

    mySemanticGraphSLAM.computeATE();
    mySemanticGraphSLAM.saveGraph();
  }
  return 0;
}

//
// Created by hd on 24/5/19.
//
#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sslam_estimator_node");

    // cameraInfoPublisher Nodelet
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load(ros::this_node::getName(), "sslam_estimator/sslam_nodelet",
                 remap, nargv);

    ros::spin();

    return 0;
}

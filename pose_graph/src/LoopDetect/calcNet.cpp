//
// Created by zh on 30/1/20.
//

#include "calcNet.h"
#include <ros/package.h>

calcNet::calcNet(){
    ImportSettings();
    PyObject * class_input = nullptr;
    std::string x;
    setenv("PYTHONPATH", this->py_path.c_str(), 1);
    x = getenv("PYTHONPATH");
    Py_Initialize();
    this->cvt = new NDArrayConverter();
    std::cout << "Module: "<< this->module_name.c_str() << std::endl;
    this->py_module = PyImport_ImportModule(this->module_name.c_str());
    std::cout << this->py_module << std::endl;
    assert(this->py_module != nullptr);
    std::cout << "Class: "<< this->class_name.c_str() << " " << this->py_module << std::endl;
    this->py_class = PyObject_GetAttrString(this->py_module, this->class_name.c_str());
    assert(this->py_class != nullptr);
    std::cout << "Creating net instance..." << this->py_class << std::endl;



    this->net = PyInstance_New(this->py_class, nullptr, nullptr);
    std::cout << "Hola" << std::endl;
    assert(this->net != nullptr);
    //std::cout << "Creating net instance..." << std::endl;
//    cv::Mat image  = cv::Mat::zeros(480,640,CV_8UC3); //Be careful with size!!
	  std::cout << "Loading net parameters..." << std::endl;
//    GetSegmentation(image);

}

void calcNet::ImportSettings(){
    std::string pkg_path = ros::package::getPath("sslam");
    std::string strSettingsFile = pkg_path + "/config/bus_av1/calc.yaml";
    cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
    py_path = pkg_path + "pose_graph/src/LoopDetect/calc/";
    fsSettings["model_path"] >> this->model_path;
    fsSettings["module_name"] >> this->module_name;
    fsSettings["class_name"] >> this->class_name;
    fsSettings["loop_detect"] >> this->loop_detect;
    fsSettings["descriptor"] >> this->descriptor;

    std::cout << "    py_path: "<< this->py_path << std::endl;
    std::cout << "    model_path: "<< this->model_path << std::endl;
    std::cout << "    module_name: "<< this->module_name << std::endl;
    std::cout << "    class_name: "<< this->class_name << std::endl;
    std::cout << "    loop_detect: "<< this->loop_detect << std::endl;
}
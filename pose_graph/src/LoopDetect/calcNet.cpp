//
// Created by zh on 30/1/20.
//

#include "calcNet.h"
//#include "Model.h"
//#include "Tensor.h"
#include <opencv2/opencv.hpp>
#include <ros/package.h>

using namespace std;
calcNet::calcNet(){
    ImportSettings();
//    std::cout << "model path: " << model_name << std::endl;
//    PyObject * class_input = nullptr;
//
//    Model m(this->model_name);
//    std::cout << "check point path: " << check_name << std::endl;

    Py_Initialize();
    PyRun_SimpleString("import sys, os");
//    PyRun_SimpleString("sys.path.append('/home/zh/catkin_ws/src/sslam_reuse/pose_graph/src/LoopDetect')");
    PyRun_SimpleString("sys.argv = ['']");
    PyRun_SimpleString("import tensorflow as tf");
    // 0. Python env setup and initialize
    std::string x;
    setenv("PYTHONPATH", this->py_path.c_str(), 1);
    x = getenv("PYTHONPATH");
    cout << "Python Path: " << x << endl;
//    m.restore(check_name);
//    std::cout << "loaded checkpoint" << std::endl;

    std::cout << "Python version: " << Py_GetVersion() << std::endl;
    this->cvt = new NDArrayConverter();

//    // py: import tensorflow
    PyObject *pName = PyUnicode_FromString("tensorflow");
    PyObject *fModule = PyImport_Import(pName);
    assert(fModule != nullptr);
    if(!fModule) std::cout << "Import tensorflow failed." << std::endl;


    PyObject *pDict = PyModule_GetDict(fModule);

    // py: print("Default graph before:", tf.get_default_graph())
    PyObject* pGetDefaultGraph = PyDict_GetItemString(pDict, "get_default_graph");
    PyObject* pDefaultGraph;
    if(PyCallable_Check(pGetDefaultGraph)){
        pDefaultGraph = PyObject_CallFunction(pGetDefaultGraph, 0);
        std::cout << "Default graph before: ";
        PyObject_Print(pDefaultGraph, stdout, 0);
        std::cout << std::endl;
    }
    else std::cout << "tensorflow.get_default_graph() is not callable.";

    // py: saver = tf.train.import_meta_graph("graph.pb.meta")
    PyObject* trainModule = PyDict_GetItemString(pDict, "train");
    PyObject* importMetaGraph = PyObject_GetAttrString(trainModule, "import_meta_graph");
    PyObject* fSaver;
    if(PyCallable_Check(importMetaGraph)){
        fSaver = PyObject_CallFunction(importMetaGraph, "(s)", "graph.pb.meta");
    }
    else std::cout << "Cannot create Tensorflow saver from imported meta graph" << std::endl;
    if(fSaver==nullptr) std::cout << "Tensorflow model failed to load from file \"" << "graph.pb.meta" << "\"" << std::endl;

    // py: sess = tf.Session()
    PyObject* session = PyDict_GetItemString(pDict, "Session");
    PyObject* fSession;
    if(PyCallable_Check(session)){
        fSession = PyObject_CallObject(session, 0);
    }
    else std::cout << "Cannot create Tensorflow session" << std::endl;
    if(fSession==nullptr) std::cout << "Tensorflow session points to zero, failed to create session" << std::endl;

    // py: saver.restore(sess, "graph.pb")
    PyObject_CallMethod(fSaver, "restore", "(0s)", fSession, "model.ckpt-200000");

    // py: print("Default graph after:", tf.get_default_graph())
    std::cout << "Default graph after: ";
    pDefaultGraph = PyObject_CallFunction(pGetDefaultGraph, 0);
    PyObject_Print(pDefaultGraph, stdout, 0);


    // 1. Module import (utils.py)
    std::cout << "Module: "<< this->module_name.c_str() << std::endl;
    this->py_module = PyImport_ImportModule(this->module_name.c_str());
    std::cout << this->py_module << std::endl;
    if (!py_module)
    {
        cerr << "[ERROR] Python get module failed." << std::endl;
    }
    std::cout << "[INFO] Python get module succeed." << std::endl;

    // 2. Class import
    std::cout << "Class: "<< this->class_name.c_str() << " in " << this->py_module << std::endl;
    this->py_class = PyObject_GetAttrString(this->py_module, this->class_name.c_str());
    if (!py_class || !PyCallable_Check(py_class))
    {
        cerr << "[ERROR] Can't find function." << endl;
    }
    cout << "[INFO] Get function (calc) succeed." << endl;

    // 3. Instance create
    std::cout << "Creating net instance..." << this->py_class << std::endl;



    this->net = PyInstance_New(this->py_class, nullptr, nullptr);
    std::cout << "Hola" << std::endl;
    assert(this->net != nullptr);
//    //std::cout << "Creating net instance..." << std::endl;
////    cv::Mat image  = cv::Mat::zeros(480,640,CV_8UC3); //Be careful with size!!
//	  std::cout << "Loading net parameters..." << std::endl;
////    GetSegmentation(image);
    Py_Finalize();
}

calcNet::~calcNet() {
    delete this->py_module;
    delete this->py_class;
    delete this->net;
    delete this->cvt;
}

void calcNet::ImportSettings(){
    std::string pkg_path = ros::package::getPath("sslam");
    std::string strSettingsFile = pkg_path + "/config/bus_av1/calc.yaml";
    cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
    py_path = pkg_path + "/pose_graph/src/LoopDetect/calc/";
    std::string local_model;
    fsSettings["model_name"] >> local_model;
    fsSettings["module_name"] >> this->module_name;
    fsSettings["class_name"] >> this->class_name;
    fsSettings["loop_detect"] >> this->loop_detect;
    fsSettings["descriptor"] >> this->descriptor;
    this->model_path = pkg_path + "/support_files/calc_model/";
    this->model_name = model_path + "saved_model.pb";
    this->check_name = model_path + local_model;
//    this->check_name = model_path + "model.ckpt-200000";
    std::cout << "    py_path: "<< this->py_path << std::endl;
    std::cout << "    model_path: "<< this->model_path << std::endl;
    std::cout << "    module_name: "<< this->module_name << std::endl;
    std::cout << "    class_name: "<< this->class_name << std::endl;
    std::cout << "    loop_detect: "<< this->loop_detect << std::endl;
}
//
// Created by zh on 30/1/20.
//

#ifndef SSLAM_CALCNET_H
#define SSLAM_CALCNET_H

#include <python2.7/Python.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <vector>
#include <cstdio>
#include "Conversion.h"

class calcNet {
private:
    NDArrayConverter *cvt; 	/*!< Converter to NumPy Array from cv::Mat */
    PyObject *py_module; 	/*!< Module of python where the Mask algorithm is implemented */
    PyObject *py_class; 	/*!< Class to be instanced */
    PyObject *net; 			/*!< Instance of the class */
    std::string py_path; 	/*!< Path to be included to the environment variable PYTHONPATH */
    std::string model_path;
    std::string model_name;
    std::string check_name;
    std::string module_name; /*!< Detailed description after the member */
    std::string class_name; /*!< Detailed description after the member */
    std::string loop_detect; 	/*!< Function of loop detection */
    std::string descriptor;

    void ImportSettings();
public:

    calcNet();
    ~calcNet();
};


#endif //SSLAM_CALCNET_H

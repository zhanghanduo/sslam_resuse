/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file Logger.hpp
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date June 2018
 * \brief SLAM main process of SSLAM-estimator.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#pragma once

#include <mutex>
#include <fstream>
#include <sstream>

class Logger
{
  public:

    Logger(Logger const&) = delete;
    void operator = (Logger const&) = delete;

    static void Write( const std::string& );

    static const std::string& FileName()
    { return instance_.filename_; }

  private:

    std::string filename_;

    std::ofstream logfile_;

    std::mutex mutex_;

    Logger();

  // Singleton instance

    static Logger instance_;
};

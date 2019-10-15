/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file timer.h
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date June 2018
 * \brief SLAM main process of SSLAM-estimator.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#pragma once

#include <chrono>

namespace utility
{
  class Timer
  {
    public:
      Timer();

      void start(); /* measures initial time */
      void stop(); /* measures elapsed time */
      double elapsed() const; /* returns elapsed time in seconds */

      static double now(); /* returns seconds since epoch */

    private:
      typedef std::chrono::high_resolution_clock clock_t;
      std::chrono::time_point<clock_t> t;
      double elapsed_seconds;
      bool started;
  };
}

std::ostream& operator<< (std::ostream& stream, const utility::Timer& t);


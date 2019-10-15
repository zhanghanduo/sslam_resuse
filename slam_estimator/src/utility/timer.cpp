/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file timer.cpp
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date June 2018
 * \brief SLAM main process of SSLAM-estimator.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#include <iomanip>
#include <ios>
#include <cassert>
#include "timer.h"

utility::Timer::Timer() :
  elapsed_seconds(0), started(false) {}

void utility::Timer::start()
{
  assert(!started);
  t = clock_t::now();
  started = true;
}

void utility::Timer::stop()
{
  assert(started);
  elapsed_seconds += std::chrono::duration<double, std::milli>(clock_t::now() - t).count() * 1e-3;
  started = false;
}

double utility::Timer::elapsed() const
{
  return elapsed_seconds;
}

double utility::Timer::now()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(clock_t::now().time_since_epoch()).count() * 1e-6;
}


std::ostream& operator<< (std::ostream& stream, const utility::Timer& t)
{
  stream << std::setprecision(16) << std::fixed << t.elapsed();
  return stream;
}

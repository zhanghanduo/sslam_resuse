/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file ScopedProfiler.hpp
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date June 2018
 * \brief SLAM main process of SSLAM-estimator.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#pragma once

#include "../timer.h"
#include "Profiler.hpp"

#include <utility>

namespace utility
{

	class ScopedProfiler
	{
	  public:

	    explicit ScopedProfiler(std::string  tag)
	      : tag_(std::move( tag ))
	    { timer_.start(); }

	    ~ScopedProfiler()
	    {
	      timer_.stop();
	      WriteToLog(tag_, timer_);
	    }

	  private:

	    const std::string tag_;

	    utility::Timer timer_;
	};

}

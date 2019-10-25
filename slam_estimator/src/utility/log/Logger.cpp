/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file Logger.cpp
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date June 2018
 * \brief SLAM main process of SSLAM-estimator.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/
#include "Logger.hpp"
#include <ros/package.h>
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <limits>

Logger Logger::instance_;

void Logger::Write(const std::string& message)
{
	std::lock_guard<std::mutex> lock( instance_.mutex_ );
	instance_.logfile_ << message;
}

std::string getTimestampedFilename()
{
	std::stringstream filename_stream;
	auto *facet = new boost::posix_time::time_facet("%Y-%m-%d_%H:%M:%S");
	filename_stream.imbue(std::locale(std::cout.getloc(), facet));
	filename_stream << boost::posix_time::second_clock::local_time() << ".log";

	return filename_stream.str();
}

std::string getTimestampedFilename(const std::string& path_)
{
	std::stringstream filename_stream;
	auto *facet = new boost::posix_time::time_facet("%Y-%m-%d_%H:%M:%S");
	filename_stream.imbue(std::locale(std::cout.getloc(), facet));
	filename_stream << path_ << boost::posix_time::second_clock::local_time() << ".log";

	return filename_stream.str();
}

Logger::Logger()
{
	std::string path = ros::package::getPath("sslam") + "/output/";
	filename_ = getTimestampedFilename(path);
	logfile_.open( filename_ );
	logfile_ << std::setprecision(std::numeric_limits<double>::digits10 + 1);
}


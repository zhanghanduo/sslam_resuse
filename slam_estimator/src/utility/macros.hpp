/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file macros.hpp
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date June 2018
 * \brief SLAM main process of SSLAM-pose_graph.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#pragma once

/** useful macros */
#define forn(i,n) for(size_t i=0;i<(n);i++)
#define fornr(i,n) for(size_t i=(n)-1;0<=i;i--)
#define forsn(i,s,n) for(size_t i=(s);i<(n);i++)
#define forsnr(i,s,n) for(size_t i=(n)-1;(s)<=i;i--)
#define forall(it,X) for(decltype((X).begin()) it=(X).begin();it!=(X).end();it++)
#define forallr(it,X) for(decltype((X).rbegin()) it=(X).rbegin();it!=(X).rend();it++)

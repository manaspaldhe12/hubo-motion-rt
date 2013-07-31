
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu> Manas Paldhe <mpaldhe@purdue.edu>
 * Date: May 30, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef WALKER_H
#define WALKER_H
#define LADDER_PLANNERINITCHAN "ladder_planner"


#include "balance-daemon.h"
#include <hubo-zmp.h>
typedef struct LadderPlanner{
	double rung_width;
	double rung_length;

	double rung_spacing;
	double first_rung_spacing;

	double rail_height;
	double rail_radius;

	double inclination;
	int number_of_stairs;


} LadderPlanner_t;

class Ladder
{

public:

    Ladder(double maxInitTime=15, double jointSpaceTolerance=0.02, double jointVelContTol=6.0);
    ~Ladder();

    ach_channel_t ladder_chan;
    ach_channel_t param_chan;
    ach_channel_t bal_cmd_chan;
    ach_channel_t bal_state_chan;

    void commenceClimbing(balance_state_t &parent_state,  balance_gains_t &gains);

    double m_jointSpaceTolerance;
    double m_jointVelContTol;
    double m_maxInitTime;

    balance_cmd_t cmd;
    balance_state_t bal_state;

    bool keepWalking;


protected:
    

    bool checkForNewTrajectory(zmp_traj_t &newTrajectory, bool haveNewTrajAlready);

    bool validateNextTrajectory( zmp_traj_element_t &current, zmp_traj_element_t &next, double dt );
    void sendState();
    void checkCommands();






};



#endif // WALKER_H


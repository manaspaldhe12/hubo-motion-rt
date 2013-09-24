
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
#ifndef LADDER_ACH_H
#define LADDER_ACH_H
#define LADDER_PLANNERINITCHAN "ladderplanner"
#define CORRECTION_PLANNERINITCHAN "correctionplanner"

typedef struct LadderPlanner{
	double rung_width;
	double rung_length;

	double rung_spacing;
	double first_rung_spacing;

	double rail_height;
	double rail_radius;

	double inclination;
	int number_of_stairs;

	double x_position;
	double y_position;
	double theta;

} LadderPlanner_t;

typedef struct CorrectionParams{

	double left_hand_x;
	double left_hand_y;
	double left_hand_z;
	double left_hand_roll;
	double left_hand_pitch;
	double left_hand_yaw;

	double right_hand_x;
	double right_hand_y;
	double right_hand_z;
	double right_hand_roll;
	double right_hand_pitch;
	double right_hand_yaw;

	double legs_x;
	double legs_y;
	double legs_z;
	double legs_yaw;

} CorrectionParams_t;




#endif//LADDER_ACH_H


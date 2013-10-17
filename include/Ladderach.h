
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
     double left_hand_x;//leftHand_xBox->value();
     double left_hand_y;//leftHand_yBox->value();
     double left_hand_z;//leftHand_zBox->value();
     double left_hand_roll;//leftHand_rollBox->value();
     double left_hand_pitch;//leftHand_pitchBox->value();
     double left_hand_yaw;//leftHand_yawBox->value();

     double right_hand_x;//rightHand_xBox->value();
     double right_hand_y;//rightHand_yBox->value();
     double right_hand_z;//rightHand_zBox->value();
     double right_hand_roll;//rightHand_rollBox->value();
     double right_hand_pitch;//rightHand_pitchBox->value();
     double right_hand_yaw;//rightHand_yawBox->value();

     double legs_x;//legs_xBox->value();
     double legs_y;//legs_yBox->value();
     double legs_z;//legs_zBox->value();
     double legs_yaw;


} CorrectionParams_t;

typedef struct TrajectoryFollowerParams{
	bool compliance_flag;//=true;
	bool pause_flag;//=false;
	bool left_hand_compliance;
	bool right_hand_compliance;
} TrajectoryFollowerParams_t;

typedef struct JointPositionCorrector{
	double joint_values[40];
} JointPositionCorrector_t;

#endif//LADDER_ACH_H



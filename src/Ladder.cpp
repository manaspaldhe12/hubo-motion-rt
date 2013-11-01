
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu> Manas Paldhe <mpaldhe@purdue.edu>
 * Date: July  29, 2013
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


#include "DrcHuboKin.h"
#include <Hubo_Control.h>
#include "Ladder.h"
#include "balance-daemon.h"

Ladder::Ladder(double maxInitTime, double jointSpaceTolerance, double jointVelContinuityTolerance) :
        m_maxInitTime(maxInitTime),
        m_jointSpaceTolerance( jointSpaceTolerance ),
        m_jointVelContTol( jointVelContinuityTolerance ),
        keepWalking(true),
	hubo()
{
    ach_status_t r = ach_open( &ladder_chan, HUBO_CHAN_LADDER_TRAJ_NAME, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                HUBO_CHAN_LADDER_TRAJ_NAME, ach_result_to_string(r), (int)r );
    
    r = ach_open( &param_chan, BALANCE_PARAM_CHAN, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                BALANCE_PARAM_CHAN, ach_result_to_string(r), (int)r );

    r = ach_open( &bal_cmd_chan, BALANCE_CMD_CHAN, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                BALANCE_CMD_CHAN, ach_result_to_string(r), (int)r );

    r = ach_open( &bal_state_chan, BALANCE_STATE_CHAN, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                BALANCE_STATE_CHAN, ach_result_to_string(r), (int)r );

    memset( &cmd, 0, sizeof(cmd) );
    memset( &bal_state, 0, sizeof(bal_state) );
}

Ladder::~Ladder()
{
    ach_close( &ladder_chan );
    ach_close( &param_chan );
    ach_close( &bal_cmd_chan );
    ach_close( &bal_state_chan );
}

void Ladder::commenceClimbing(balance_state_t &parent_state, balance_gains_t &gains)
{
    std::cerr << "In commence climbing" << std::endl;
    fflush(stderr);
    
    int timeIndex=0, nextTimeIndex=0, prevTimeIndex=0;
    size_t fs;
 
    zmp_traj_t *currentTrajectory, *prevTrajectory, *nextTrajectory;
    currentTrajectory = new zmp_traj_t;
    prevTrajectory = new zmp_traj_t;
    nextTrajectory = new zmp_traj_t;
    memset( currentTrajectory, 0, sizeof(*currentTrajectory) );
    memset( prevTrajectory, 0, sizeof(*prevTrajectory) );
    memset( nextTrajectory, 0, sizeof(*nextTrajectory) );
    TrajectoryFollowerParams_t traj_params;
    memset(&traj_params, 0, sizeof(traj_params)); 
    // TODO: Consider making these values persistent
    //memset( &state, 0, sizeof(state) );

    memcpy( &bal_state, &parent_state, sizeof(bal_state) );

    bal_state.m_balance_mode = BAL_LADDER_CLIMBING;
    sendState();

    fprintf(stdout, "Waiting for first trajectory\n"); fflush(stdout);
    ach_status_t r;
    do {
        struct timespec t;
        clock_gettime( ACH_DEFAULT_CLOCK, &t );
        t.tv_sec += 1;
        r = ach_get( &ladder_chan, currentTrajectory, sizeof(*currentTrajectory), &fs,
                    &t, ACH_O_WAIT | ACH_O_LAST );
  	std::cout << "ladder traj ach_get result: " << ach_result_to_string(r) << "\n";

        checkCommands();
    } while(!daemon_sig_quit  && r==ACH_TIMEOUT); 

    if(!daemon_sig_quit)
        fprintf(stdout, "First trajectory acquired\n");
    
        
    daemon_assert( !daemon_sig_quit, __LINE__ );

    ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );

    hubo.update(true);
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        if( LF1!=i && LF2!=i && LF3!=i && LF4!=i && LF5!=i
         && RF1!=i && RF2!=i && RF3!=i && RF4!=i && RF5!=i
         && NK1!=i && NK2!=i && NKY!=i )
        {
            hubo.setJointAngle( i, currentTrajectory->traj[0].angles[i] );
            hubo.setJointNominalSpeed( i, 0.4 );
            hubo.setJointNominalAcceleration( i, 0.4 );
        }
    }
    
    hubo.setJointNominalSpeed( RKN, 0.8 );
    hubo.setJointNominalAcceleration( RKN, 0.8 );
    hubo.setJointNominalSpeed( LKN, 0.8 );
    hubo.setJointNominalAcceleration( LKN, 0.8 );

    hubo.sendControls();

    double m_maxInitTime = 10;
    double biggestErr = 0;
    int worstJoint=-1;
    
    double dt, time, stime; stime=hubo.getTime(); time=hubo.getTime();
    double norm = m_jointSpaceTolerance+1; // make sure this fails initially
    while( !daemon_sig_quit && (norm > m_jointSpaceTolerance && time-stime < m_maxInitTime)) {
        hubo.update(true);
        norm = 0;
        for(int i=0; i<HUBO_JOINT_COUNT; i++)
        {
            double err=0;
            if( LF1!=i && LF2!=i && LF3!=i && LF4!=i && LF5!=i
             && RF1!=i && RF2!=i && RF3!=i && RF4!=i && RF5!=i
             && NK1!=i && NK2!=i && NKY!=i )
                err = (hubo.getJointAngleState( i )-currentTrajectory->traj[0].angles[i]);
            norm += err*err;
            if( fabs(err) > fabs(biggestErr) )
            {
                biggestErr = err;
                worstJoint = i;
            }
        }
        time = hubo.getTime();
    }

    if( time-stime >= m_maxInitTime )
    {
        fprintf(stderr, "Warning: could not reach the starting Trajectory within %f seconds\n"
                        " -- Biggest error was %f radians in joint %s\n",
                        m_maxInitTime, biggestErr, jointNames[worstJoint] );
    }

    timeIndex = 1;
    bool haveNewTrajectory = false;

    int test_counter=0;
    bool increasing=true;

    while(!daemon_sig_quit)
    {
        haveNewTrajectory = checkForNewTrajectory(*nextTrajectory, haveNewTrajectory);
        ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );
        hubo.update(true);
	printf("timeindex is %d \n", timeIndex);

        dt = hubo.getTime() - time;
        time = hubo.getTime();
        /*if( dt <= 0 )
        {
            fprintf(stderr, "Something unnatural has happened... %f\n", dt);
            continue;
        }*/

        if( timeIndex==0 )
        {
            nextTimeIndex = timeIndex+1;
            //executeTimeStep( hubo, prevTrajectory->traj[prevTimeIndex],
            //                       currentTrajectory->traj[timeIndex],
            //                       currentTrajectory->traj[nextTimeIndex],
            //                       gains, dt );
            
        }
        else if( timeIndex == currentTrajectory->periodEndTick && haveNewTrajectory )
        { // Note: This should never happen
            if( validateNextTrajectory( currentTrajectory->traj[timeIndex],
                                        nextTrajectory->traj[0], dt ) )
            {
                nextTimeIndex = 0;
              //  executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
              //                         currentTrajectory->traj[timeIndex],
              //                         nextTrajectory->traj[nextTimeIndex],
              //                         gains, dt );
                
                memcpy( prevTrajectory, currentTrajectory, sizeof(*prevTrajectory) );
                memcpy( currentTrajectory, nextTrajectory, sizeof(*nextTrajectory) );
                fprintf(stderr, "Notice: Swapping in new trajectory\n");
            }
            else
            {
                fprintf(stderr, "WARNING: Discontinuous trajectory passed in. Stop.\n");
                bal_state.m_walk_error = WALK_FAILED_SWAP;

                nextTimeIndex = timeIndex+1;
            //    executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
            //                           currentTrajectory->traj[timeIndex],
            //                           currentTrajectory->traj[nextTimeIndex],
            //                           gains, dt );
            }
            haveNewTrajectory = false;
        }
        else if( timeIndex == currentTrajectory->periodEndTick && currentTrajectory->reuse )
        { // Note: This should also never happen
            fprintf(stderr, "You have commanded me to reuse a trajectory!\n");
            checkCommands();
            if( cmd.cmd_request != BAL_ZMP_WALKING )
                currentTrajectory->reuse = false;

            if( currentTrajectory->reuse == true )
                nextTimeIndex = currentTrajectory->periodStartTick;
            else
                nextTimeIndex = timeIndex+1;

         //   executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
         //                          currentTrajectory->traj[timeIndex],
         //                          currentTrajectory->traj[nextTimeIndex],
         //                          gains, dt );
        }
        else if( timeIndex < currentTrajectory->count-1 )
        {
	   r = ach_open(&traj_params_chan, HUBO_CHAN_TRAJECTORY_PARAMS, NULL );
	   if( r != ACH_OK )
      		fprintf( stderr, "Problem with channel %s: %s (%d)\n", HUBO_CHAN_TRAJECTORY_PARAMS, ach_result_to_string(r), (int)r );
	  
           ach_get( &traj_params_chan, &traj_params, sizeof(traj_params), &fs, NULL, ACH_O_LAST );

	    while (traj_params.pause_flag==true){
		ach_get( &traj_params_chan, &traj_params, sizeof(traj_params), &fs, NULL, ACH_O_LAST );
	   	printf("paused now \n");
	     }	   
 
            nextTimeIndex = timeIndex+1;
            printf(" in this step \n");
            executeTimeStepCompliance( hubo,currentTrajectory->traj[prevTimeIndex],
                                   currentTrajectory->traj[timeIndex],
                                   currentTrajectory->traj[nextTimeIndex],
                                   gains, dt, traj_params.compliance_flag, traj_params.left_hand_compliance, traj_params.right_hand_compliance );
            printf("executed a step \n");
	    fflush(stdout);
        }
        else
        {
            checkCommands();
        }

        prevTimeIndex = timeIndex;
        timeIndex = nextTimeIndex;
        sendState();
    }

    sendState();
}


void Ladder::changeJointAngles(double joint_positions[], Hubo_Control &hubo){

	double current_angles[HUBO_JOINT_COUNT];
	for (int interpolate=0; interpolate<10;  interpolate++){
		for (int joint=0; joint<HUBO_JOINT_COUNT; joint++){
			double start_angle = hubo.getJointAngle(joint);
			double end_angle = joint_positions[joint];
			current_angles[joint]=start_angle + interpolate*(end_angle-start_angle)/10;
			hubo.setJointAngle(joint, current_angles[joint]);
			printf("joint %d is going to %lf \n", joint,current_angles[joint]);
		}	
		//hubo.sendControls();
	}

}


void Ladder::commenceCorrection(balance_state_t &parent_state, balance_gains_t &gains)
{
    std::cerr << "In commence climbing" << std::endl;
    fflush(stderr);
    
    int timeIndex=0, nextTimeIndex=0, prevTimeIndex=0;
    size_t fs;
 
    zmp_traj_t *currentTrajectory, *prevTrajectory, *nextTrajectory;
    currentTrajectory = new zmp_traj_t;
    prevTrajectory = new zmp_traj_t;
    nextTrajectory = new zmp_traj_t;
    memset( currentTrajectory, 0, sizeof(*currentTrajectory) );
    memset( prevTrajectory, 0, sizeof(*prevTrajectory) );
    memset( nextTrajectory, 0, sizeof(*nextTrajectory) );
    
    // TODO: Consider making these values persistent
    //memset( &state, 0, sizeof(state) );

    memcpy( &bal_state, &parent_state, sizeof(bal_state) );

    bal_state.m_balance_mode = BAL_LADDER_CLIMBING;
    sendState();

    fprintf(stdout, "Waiting for first trajectory\n"); fflush(stdout);
    ach_status_t r;
    do {
        struct timespec t;
        clock_gettime( ACH_DEFAULT_CLOCK, &t );
        t.tv_sec += 1;
        r = ach_get( &ladder_chan, currentTrajectory, sizeof(*currentTrajectory), &fs,
                    &t, ACH_O_WAIT | ACH_O_LAST );
  	std::cout << "ladder traj ach_get result: " << ach_result_to_string(r) << "\n";

        checkCommands();
    } while(!daemon_sig_quit  && r==ACH_TIMEOUT); 

    if(!daemon_sig_quit)
        fprintf(stdout, "First trajectory acquired\n");
    
        
    daemon_assert( !daemon_sig_quit, __LINE__ );

    ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );

    hubo.update(true);
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        if( LF1!=i && LF2!=i && LF3!=i && LF4!=i && LF5!=i
         && RF1!=i && RF2!=i && RF3!=i && RF4!=i && RF5!=i
         && NK1!=i && NK2!=i && NKY!=i )
        {
            hubo.setJointAngle( i, currentTrajectory->traj[0].angles[i] );
            hubo.setJointNominalSpeed( i, 0.4 );
            hubo.setJointNominalAcceleration( i, 0.4 );
        }
    }
    
    hubo.setJointNominalSpeed( RKN, 0.8 );
    hubo.setJointNominalAcceleration( RKN, 0.8 );
    hubo.setJointNominalSpeed( LKN, 0.8 );
    hubo.setJointNominalAcceleration( LKN, 0.8 );

    hubo.sendControls();

    double m_maxInitTime = 10;
    double biggestErr = 0;
    int worstJoint=-1;
    
    double dt, time, stime; stime=hubo.getTime(); time=hubo.getTime();
    double norm = m_jointSpaceTolerance+1; // make sure this fails initially
    while( !daemon_sig_quit && (norm > m_jointSpaceTolerance && time-stime < m_maxInitTime)) {
        hubo.update(true);
        norm = 0;
        for(int i=0; i<HUBO_JOINT_COUNT; i++)
        {
            double err=0;
            if( LF1!=i && LF2!=i && LF3!=i && LF4!=i && LF5!=i
             && RF1!=i && RF2!=i && RF3!=i && RF4!=i && RF5!=i
             && NK1!=i && NK2!=i && NKY!=i )
                err = (hubo.getJointAngleState( i )-currentTrajectory->traj[0].angles[i]);
            norm += err*err;
            if( fabs(err) > fabs(biggestErr) )
            {
                biggestErr = err;
                worstJoint = i;
            }
        }
        time = hubo.getTime();
    }

    if( time-stime >= m_maxInitTime )
    {
        fprintf(stderr, "Warning: could not reach the starting Trajectory within %f seconds\n"
                        " -- Biggest error was %f radians in joint %s\n",
                        m_maxInitTime, biggestErr, jointNames[worstJoint] );
    }

    timeIndex = 1;
    bool haveNewTrajectory = false;

    int test_counter=0;
    bool increasing=true;

    while(!daemon_sig_quit)
    {
        haveNewTrajectory = checkForNewTrajectory(*nextTrajectory, haveNewTrajectory);
        ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );
        hubo.update(true);
	printf("timeindex is %d \n", timeIndex);

        dt = hubo.getTime() - time;
        time = hubo.getTime();
        /*if( dt <= 0 )
        {
            fprintf(stderr, "Something unnatural has happened... %f\n", dt);
            continue;
        }*/

        if( timeIndex==0 )
        {
            nextTimeIndex = timeIndex+1;
            //executeTimeStep( hubo, prevTrajectory->traj[prevTimeIndex],
            //                       currentTrajectory->traj[timeIndex],
            //                       currentTrajectory->traj[nextTimeIndex],
            //                       gains, dt );
            
        }
        else if( timeIndex == currentTrajectory->periodEndTick && haveNewTrajectory )
        { // Note: This should never happen
            if( validateNextTrajectory( currentTrajectory->traj[timeIndex],
                                        nextTrajectory->traj[0], dt ) )
            {
                nextTimeIndex = 0;
              //  executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
              //                         currentTrajectory->traj[timeIndex],
              //                         nextTrajectory->traj[nextTimeIndex],
              //                         gains, dt );
                
                memcpy( prevTrajectory, currentTrajectory, sizeof(*prevTrajectory) );
                memcpy( currentTrajectory, nextTrajectory, sizeof(*nextTrajectory) );
                fprintf(stderr, "Notice: Swapping in new trajectory\n");
            }
            else
            {
                fprintf(stderr, "WARNING: Discontinuous trajectory passed in. Stop.\n");
                bal_state.m_walk_error = WALK_FAILED_SWAP;

                nextTimeIndex = timeIndex+1;
            //    executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
            //                           currentTrajectory->traj[timeIndex],
            //                           currentTrajectory->traj[nextTimeIndex],
            //                           gains, dt );
            }
            haveNewTrajectory = false;
        }
        else if( timeIndex == currentTrajectory->periodEndTick && currentTrajectory->reuse )
        { // Note: This should also never happen
            fprintf(stderr, "You have commanded me to reuse a trajectory!\n");
            checkCommands();
            if( cmd.cmd_request != BAL_ZMP_WALKING )
                currentTrajectory->reuse = false;

            if( currentTrajectory->reuse == true )
                nextTimeIndex = currentTrajectory->periodStartTick;
            else
                nextTimeIndex = timeIndex+1;

         //   executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
         //                          currentTrajectory->traj[timeIndex],
         //                          currentTrajectory->traj[nextTimeIndex],
         //                          gains, dt );
        }
        else if( timeIndex < currentTrajectory->count-1 )
        {
	   
            nextTimeIndex = timeIndex+1;
            printf(" in this step \n");
            executeCorrectionStep( hubo,currentTrajectory->traj[prevTimeIndex],
                                   currentTrajectory->traj[timeIndex],
                                   currentTrajectory->traj[nextTimeIndex],
                                   gains, dt );
            printf("executed a step \n");
	    fflush(stdout);
        }
        else
        {
            checkCommands();
        }

        prevTimeIndex = timeIndex;
        timeIndex = nextTimeIndex;
        sendState();
    }

    sendState();
}



void Ladder::executeTestStep(Hubo_Control &hubo, int counter){

     for (int i=0; i<HUBO_JOINT_COUNT; i++){
        if (i==LEB){
                printf("%f , \n ", ((float) counter)/1000);
                hubo.setJointAngle(i, ((float) counter)/1000);
        }
        else{
                hubo.setJointAngle(i,0);
        }
     }
     hubo.sendControls();
}


bool Ladder::checkForNewTrajectory(zmp_traj_t &newTrajectory, bool haveNewTrajAlready)
{
    size_t fs;
    
    ach_status_t r = ach_get( &ladder_chan, &newTrajectory, sizeof(newTrajectory), &fs, NULL, ACH_O_LAST );

    if( ACH_OK==r || ACH_MISSED_FRAME==r )
    {
        fprintf(stdout, "Noticed new trajectory: ID #%d\n", (int)newTrajectory.trajNumber);
        return true;
    }
    else
        return haveNewTrajAlready || false;

}

bool Ladder::validateNextTrajectory( zmp_traj_element_t &current, zmp_traj_element_t &next, double dt )
{
    bool valid = true;
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        if( fabs(next.angles[i]-current.angles[i])/fabs(dt) > fabs(m_jointVelContTol) )
            valid = false;

    return valid;
}


void Ladder::executeTimeStep(Hubo_Control &hubo, zmp_traj_element_t &prevElem,
            zmp_traj_element_t &currentElem, zmp_traj_element &nextElem,
            balance_gains_t &gains, double dt )
{
    double vel, accel;

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
	  printf("%f , \n",currentElem.angles[i]);
   	  hubo.setJointTraj( i, currentElem.angles[i], 0);
    }
    hubo.sendControls();
    printf("out of the loop \n");
}

void Ladder::executeTimeStepCompliance(Hubo_Control &hubo, zmp_traj_element_t &prevElem,
            zmp_traj_element_t &currentElem, zmp_traj_element &nextElem,
            balance_gains_t &gains, double dt, bool compliance_flag, bool left_hand_compliance, bool right_hand_compliance)
{
    double vel, accel;

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
	 printf("%f , \n",currentElem.angles[i]);
	 if (compliance_flag ==false && left_hand_compliance==false && right_hand_compliance ==false){
	        hubo.setJointCompliance(i, false);//not arms or compliance flags are all false
	   	hubo.setJointAngle( i, currentElem.angles[i]);
	  }	 
   	  else{
		if (compliance_flag || left_hand_compliance){
                        hubo.setArmAntiFriction(LEFT, true);
                        hubo.setArmCompliance(LEFT, true); // These will turn on compliance with the default gains of hubo-ach		
		}
	  	else if ((compliance_flag || right_hand_compliance) && (i>=11) && (i<=17)){
	  	        hubo.setArmAntiFriction(RIGHT, true);
                        hubo.setArmCompliance(RIGHT, true);
		}

                DrcHuboKin kin;
                kin.updateHubo(hubo);

                ArmVector torques; // Vector to hold expected torques due to gravity
                double time, dt=0;
                time = hubo.getTime();
                double qlast[HUBO_JOINT_COUNT]; // Array of the previous reference commands for all the joints (needed to calculate velocity)
                for(int i=0; i<HUBO_JOINT_COUNT; i++){
                         qlast[i] = hubo.getJointAngle(i);
                }

                hubo.update();
                kin.updateHubo(hubo);
                dt = hubo.getTime() - time;
                time = hubo.getTime();

                if (compliance_flag || left_hand_compliance){
                      kin.armTorques(LEFT, torques);
                      hubo.setArmTorques(LEFT, torques);
                }

                if (compliance_flag || right_hand_compliance){
                      kin.armTorques(RIGHT, torques);
                      hubo.setArmTorques(RIGHT, torques);
                }
                hubo.setJointTraj(i, currentElem.angles[i], ((currentElem.angles[i]-qlast[i])/dt));
	  }
   }
    hubo.sendControls();
    if (compliance_flag==true){
	printf("compliance is on \n");
    }
    else {
	printf("compliance is off \n");
    }

    if (left_hand_compliance==true){
	printf("left compliance is on \n");
    }
    else {
	printf("left compliance is off \n");
    }

    if (right_hand_compliance==true){
	printf("right compliance is on \n");
    }
    else {
	printf("right compliance is off \n");
    }


}


void Ladder::executeCorrectionStep(Hubo_Control &hubo, zmp_traj_element_t &prevElem,
            zmp_traj_element_t &currentElem, zmp_traj_element &nextElem,
            balance_gains_t &gains, double dt )
{
    double vel, accel;

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
	  if (i==LEB){
	  	printf("%f , \n",currentElem.angles[i]);
		hubo.setJointTraj( i, currentElem.angles[i], 0);
	  }
	  else{
 		hubo.setJointAngle( i, 0);
		//send 0
	  }
    }
    hubo.sendControls();
    printf("out of the loop \n");
}


void Ladder::sendState()
{
    ach_put( &bal_state_chan, &bal_state, sizeof(bal_state) );
}


void Ladder::checkCommands()
{
    size_t fs;
    ach_get( &bal_cmd_chan, &cmd, sizeof(cmd), &fs, NULL, ACH_O_LAST );
}


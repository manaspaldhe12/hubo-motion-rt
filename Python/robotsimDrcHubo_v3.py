#!/usr/bin/env python
#// This program is free software: you can redistribute it and/or modify
#// it under the terms of the GNU Lesser General Public License as published by
#// the Free Software Foundation, either version 3 of the License, or
#// at your option) any later version.
#//
#// This program is distributed in the hope that it will be useful,
#// but WITHOUT ANY WARRANTY; without even the implied warranty of
#// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#// GNU Lesser General Public License for more details.
#//
#// You should have received a copy of the GNU Lesser General Public License
#// along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import with_statement # for python 2.5
__author__ = 'Kris Hauser'
__license__ = 'GPLv3 license'

import hubo_ach as ha
import ach
import time
from ctypes import *

from optparse import OptionParser

import math
import time
import sys
#RobotSim python module
from robotsim.robot import WorldModel,Simulator
#OpenGL visualizer
from robotsim.robot.glprogram import *


skip = 100
skipi = 0
skiptemp = 0.0


class Timer(object):
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        global skip
        global skipi
        global skiptemp
        skiptemp = skiptemp + (time.time() - self.tstart)
        if (skipi < skip):
            skipi = skipi + 1
        else:
            skiptemp = skiptemp/100.0
            if self.name:
                print '[%s]' % self.name,
           # print 'Elapsed: %s' % (time.time() - self.tstart)
            print 'Elapsed: ',skiptemp,' sec : ', (ha.HUBO_LOOP_PERIOD/skiptemp * 100.0),' percent'
            skipi = 0

#setup the index mappings between hubo ach and robotsim
#what about NK2?
#rs_index2 is the link index in the whole body index in robotsim 
ha_fingerL = [ha.LF1,ha.LF1,ha.LF1,ha.LF1,ha.LF1,ha.LF1,ha.LF1,ha.LF1,ha.LF1]
rs_fingerL = range(7,16)
rs_fingerL2 = range(13,22)
ha_fingerR = [ha.RF1,ha.RF1,ha.RF1,ha.RF1,ha.RF1,ha.RF1,ha.RF1,ha.RF1,ha.RF1,ha.RF1,ha.RF1,ha.RF1]
rs_fingerR = range(26,38)
rs_fingerR2 = range(33,45)
ha_neck = [ha.NKY,ha.NK1,ha.NK2]
rs_neck = [16,17,18]
rs_neck2 = [23,24,25]
ha_rarm = [ha.RSP,ha.RSR,ha.RSY,ha.REB,ha.RWY,ha.RWP,ha.RWR]
rs_rarm = range(19,26)
rs_rarm2 = range(26,33)
ha_larm = [ha.LSP,ha.LSR,ha.LSY,ha.LEB,ha.LWY,ha.LWP,ha.LWR]
rs_larm = range(0,6)
rs_larm2 = range(6,13)
ha_wst = ha.WST
rs_wst = 38
rs_wst2 = 46
ha_rleg = [ha.RHY,ha.RHR,ha.RHP,ha.RKN,ha.RAP,ha.RAR]
rs_rleg = range(45,51)
rs_rleg2 = range(54,60)
ha_lleg = [ha.LHY,ha.LHR,ha.LHP,ha.LKN,ha.LAP,ha.LAR]
rs_lleg = range(39,45)
rs_lleg2 = range(47,53)
joint_mapping = zip(ha_fingerR,rs_fingerR)+zip(ha_fingerL,rs_fingerL)+zip(ha_neck,rs_neck)+zip(ha_rarm,rs_rarm)+zip(ha_larm,rs_larm)+[(ha_wst,rs_wst)]+zip(ha_rleg,rs_rleg)+zip(ha_lleg,rs_lleg)
encoder_joint_mapping = zip(ha_fingerR,rs_fingerR2)+zip(ha_fingerL,rs_fingerL2)+zip(ha_neck,rs_neck2)+zip(ha_rarm,rs_rarm2)+zip(ha_larm,rs_larm2)+[(ha_wst,rs_wst2)]+zip(ha_rleg,rs_rleg2)+zip(ha_lleg,rs_lleg2)
max_rs_index = 51
max_rs_index2 = 61

def MakeSimulator(filename):
    world = WorldModel()
    if not world.readFile(filename):
        raise ValueError("Unable to read setup model file")
    if len(world.robot(0).getConfig()) != 61:
        raise ValueError("Robot doesn't seem to have the right number of DOFs")
    sim = Simulator(world)
    robot = sim.getController(0)

    #test if its the right robot with the right set of sensors
    sensors = ["encoders","TorsoTilt","LF_Tilt","RF_Tilt","RH_ForceSensor","LH_ForceSensor","RF_ForceSensor","LF_ForceSensor"]
    for s in sensors:
        t = robot.getNamedSensor(s)
        if t.name()=='':
            raise ValueError("World XML file should have sensor "+s+" set up")

    sim.encoder = robot.getNamedSensor("encoders")
    sim.imus = [robot.getNamedSensor("TorsoTilt"),
                      robot.getNamedSensor("LF_Tilt"),
                      robot.getNamedSensor("RF_Tilt")]
    sim.fts = [robot.getNamedSensor("RH_ForceSensor"),
                     robot.getNamedSensor("LH_ForceSensor"),
                     robot.getNamedSensor("RF_ForceSensor"),
                     robot.getNamedSensor("LF_ForceSensor")]
    sim.timestep = 0.005
    return sim

def GetSimulationState(simulator,state):
    """Sets the hubo-ach sensor state from the simulation values"""
    pose = simulator.encoder.getMeasurements()
    imus = [imu.getMeasurements() for imu in simulator.imus]
    fts = [ft.getMeasurements() for ft in simulator.fts]

    print "-----simulation state-----" + str(len(pose))
    for (ha_ind,rs_ind) in encoder_joint_mapping:
        state.joint[ha_ind].pos = pose[rs_ind]
        print 'REB:'+str(pose[37])
#        if (ha_ind >= ha.RSP and ha_ind <= ha.RWP):
#            print 'right arm: ' + str(ha_ind) +', ' + str(rs_ind) +', ' + str(pose[rs_ind]) 
#        if (ha_ind >= ha.LSP and ha_ind <= ha.LWP):
#            print 'left arm: ' + str(ha_ind) +', '  + str(rs_ind) +', '  + str(pose[rs_ind]) 
#        if (ha_ind >= ha.RHY and ha_ind <= ha.RAR):
#            print 'right leg: ' + str(ha_ind) +', ' + str(rs_ind) +', '  + str(pose[rs_ind]) 
#        if (ha_ind >= ha.LHY and ha_ind <= ha.LAR):
#            print 'left leg: ' + str(ha_ind) +', '  + str(rs_ind) +', '  + str(pose[rs_ind])             

    #fill out IMUs
    for i,imuvals in enumerate(imus):
        state.imu[i].a_x = imuvals[0]
        state.imu[i].a_y = imuvals[1]
        state.imu[i].a_z = imuvals[2]
        state.imu[i].w_x = imuvals[3]
        state.imu[i].w_y = imuvals[4]
        state.imu[i].w_z = imuvals[5]

    #fill out FTs
    for i,ftvals in enumerate(fts):
        state.ft[i].m_x = ftvals[3]
        state.ft[i].m_y = ftvals[4]
        state.ft[i].f_z = ftvals[2]

def SendDesiredConfig(simulation, ref, prevRef, state):
    """Sets the hubo-ach cmd state to the simulation controller"""
    qdes = [0]*max_rs_index
    dqdes = [0]*max_rs_index
    curdes = [0]*max_rs_index
#    print "-----Reference state-----"
    for (ha_ind,rs_ind) in joint_mapping:
#        if ha_ind >= len(state.joint):
#            print "Warning, hubo-ach index",ha_ind,"out of bounds"
#            continue
        qdes[rs_ind] = ref.ref[ha_ind]
        dqdes[rs_ind] = (ref.ref[ha_ind]-prevRef.ref[ha_ind])/simulation.timestep
#        if dqdes[rs_ind] < 0.00000001:
#            dqdes[rs_ind] = (ref.ref[ha_ind] - state.joint[ha_ind].pos)/simulation.timestep
        
#        if (ha_ind >= ha.RSP and ha_ind <= ha.RWP):
#            print 'right arm: ' + str(ha_ind) +', '  + str(rs_ind) +', '  + str(ref.ref[ha_ind]) +"," + str(dqdes[rs_ind])
#        if (ha_ind >= ha.LSP and ha_ind <= ha.LWP):
#            print 'left arm: ' + str(ha_ind) +', '  + str(rs_ind) +', '  + str(ref.ref[ha_ind])  +"," + str(dqdes[rs_ind])
#        if (ha_ind >= ha.RHY and ha_ind <= ha.RAR):
#            print 'right leg: ' + str(ha_ind) +', '  + str(rs_ind) +', '  + str(ref.ref[ha_ind])  +"," + str(dqdes[rs_ind])
#        if (ha_ind >= ha.LHY and ha_ind <= ha.LAR):
#            print 'left leg: ' + str(ha_ind) +', '  + str(rs_ind) +', '  + str(ref.ref[ha_ind]) +"," + str(dqdes[rs_ind])

        #curdes[rs_ind] = state.joint[ha_ind].cur
    
#    print "-----qdes:-----" + str(qdes)
#    print "-----dqdes:-----" + str(dqdes)
    
    # Can this be replaced with something that only takes qdes?
    # Technically, the hubo hardware only receives position instructions
    simulation.getController(0).setPIDCommand(qdes,dqdes)

def StepSimulation(simulation):
    simulation.simulate(simulation.timestep)

def DrawSimulation(simulation):
    simulation.updateWorld()
    simulation.getWorld().drawGL()




class RobotSimHuboAchProgram(GLRealtimeProgram):
    def __init__(self,worldfn):
        GLRealtimeProgram.__init__(self,"Hubo-ach virtual robot server")
        self.simulator = MakeSimulator(worldfn) 

        # Hubo-Ach Start and setup:
        self.stateChannel = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
        self.stateChannel.flush()
        self.state = ha.HUBO_STATE()
        self.state.time = 0

        self.refChannel = ach.Channel(ha.HUBO_CHAN_REF_NAME)
        self.refChannel.flush()
        self.ref = ha.HUBO_REF()
        self.prevRef = ha.HUBO_REF()
    
#        self.timeChannel = ach.Channel(ha.HUBO_CHAN_VIRTUAL_TO_SIM_NAME)
#        self.timeChannel.flush()
        
        self.sim = ha.HUBO_VIRTUAL()
    
        self.feedbackChannel = ach.Channel(ha.HUBO_CHAN_VIRTUAL_FROM_SIM_NAME)
        self.feedbackChannel.flush()

        #self.feedbackChannel.put(self.sim)

    def display(self):
        DrawSimulation(self.simulator)

    def idle(self):
        #KH question: what does this line do?
#        [statuss, framesizes] = self.timeChannel.get(self.sim, wait=True, last=False)

        #[statuss, framesizes] = self.stateChannel.get(self.state, wait=False, last=True)
        [statuss, framesizes] = self.refChannel.get(self.ref, wait=False, last=True)
        
        print "self.state.time"
        print self.state.time
        
#        if self.state.time > 0.0:
#                print("something")
#                wait = input("PRESS ENTER TO CONTINUE.")
#                print("something")

        # Set Reference from simulation
        SendDesiredConfig(self.simulator,self.ref,self.prevRef, self.state)
        #print self.state
          
        # this will step the simulation  note: i can run env step in a loop if nothign else changes

        #N = math.ceil(ha.HUBO_LOOP_PERIOD/self.simulator.timestep)
        #T = 1/N*ha.HUBO_LOOP_PERIOD
        #        print 'openhubo.TIMESTEP = ',openhubo.TIMESTEP, ' : N = ', N, ' : T = ', T
        #for x in range(0,int(N)):
        StepSimulation(self.simulator)  # this is in seconds
        #self.sim.time = self.sim.time + self.simulator.timestep
        self.state.time = self.state.time + self.simulator.timestep
        GetSimulationState(self.simulator,self.state)
        self.state.refWait = 0
        self.stateChannel.put(self.state)
#        print ha.HUBO_CHAN_STATE_NAME
        #self.feedbackChannel.put(self.sim) 
        
        for (ha_ind,rs_ind) in joint_mapping:
            self.prevRef.ref[ha_ind] = self.ref.ref[ha_ind]
        
        glutPostRedisplay()
        self.sim.time = self.state.time
        self.feedbackChannel.put(self.sim) 

if __name__=='__main__':

    parser = OptionParser()
#    parser.add_option("-n", "--file", dest="filename",
#        help="write report to FILE", metavar="FILE")
#    parser.add_option("-q", "--quiet",
#        action="store_false", dest="verbose", default=True,
#        help="don't print status messages to stdout")

    (options, args) = parser.parse_args()


#    print 'options: ', options
#    print 'args: ', args[0], ' : num args = ',len(args)

    try:
        sim_fn = args[0]
    except:
        sim_fn = "BootCamp/hubo_plane_ach_drc_v3.xml"

    print "Launching with simulation file",sim_fn
    RobotSimHuboAchProgram(sim_fn).run()


# end here

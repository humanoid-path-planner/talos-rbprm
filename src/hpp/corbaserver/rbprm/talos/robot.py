#!/usr/bin/env python
# Copyright (c) 2019 CNRS
# Author: Pierre Fernbach
#
# This file is part of hpp-rbprm-robot-data.
# hpp_tutorial is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp_tutorial is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp_tutorial.  If not, see
# <http://www.gnu.org/licenses/>.

from hpp.corbaserver.rbprm.rbprmfullbody import FullBody as Parent
from pinocchio import SE3, Quaternion
import numpy as np

class Robot (Parent):
    ##
    #  Information to retrieve urdf and srdf files.
    packageName = "talos_data"
    meshPackageName = "talos_data"
    rootJointType = "freeflyer"    
    urdfName = "talos"
    urdfSuffix = "_reduced"
    srdfSuffix = ""

    ## Information about the names of thes joints defining the limbs of the robot
    rLegId = 'talos_rleg_rom'
    rleg = 'leg_right_1_joint'
    rfoot = 'leg_right_6_joint'

    lLegId = 'talos_lleg_rom'
    lleg = 'leg_left_1_joint'
    lfoot = 'leg_left_6_joint'

    rArmId = 'talos_rarm_rom'
    rarm = 'arm_right_1_joint'
    rhand = 'arm_right_7_joint'

    lArmId = 'talos_larm_rom'
    larm = 'arm_left_1_joint'
    lhand = 'arm_left_7_joint'

    referenceConfig = [
        0.0, 0.0,  1.0232773,  0.0 ,  0.0, 0.0, 1.,                       #Free flyer
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,            #Left Leg
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,            #Right Leg
        0.0 ,  0.006761,                                                  #Chest
        0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1,-0.005,  #Left Arm
        -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,#Right Arm
        0.,  0.];

    referenceConfig_legsApart =[
        0.0, 0.0,  1.0232773,  0.0 ,  0.0, 0.0, 1.,                       #Free flyer
        0.0,  0.06, -0.411354,  0.859395, -0.448041, -0.061708,           #Left Leg
        0.0,  -0.06, -0.411354,  0.859395, -0.448041, 0.058292,           #Right Leg
        0.0 ,  0.006761,                                                  #Chest
        0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1,-0.005,  #Left Arm
        -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,#Right Arm
        0.,  0.]

    referenceConfig_armsFront =[
        0.0, 0.0,  1.0232773,  1 ,  0.0, 0.0, 0.0,                     #Free flyer
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,         #Left Leg
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,         #Right Leg
        0.0 ,  0.006761,                                               #Chest
        -0.5 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1,-0.005,  #Left Arm
        0.5 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,  #Right Arm
        0.,  0.]

    referenceConfig_legsSide=[
        0.0, 0.0, 0.9832773, 1, 0.0, 0.0, 0.0,                      #Free flyer
        1.57, 0.0, -0.611354, 1.059395, -0.448041,-0.001708,        #Left Leg
        -1.57, 0.0, -0.611354, 1.059395, -0.448041, -0.001708,      #Right Leg
         0.0, 0.006761,                                             #Chest
         -0.5,0.173046, -0.0002, -0.525366, 0.0, -0.0, 0.1, -0.005, #Left Arm
        0.5, -0.173046, 0.0002, -0.525366, 0.0, 0.0, 0.1, -0.005,   #Right Arm
        0.0, 0.0]

    # informations required to generate the limbs databases : 
    nbSamples = 50000
    octreeSize = 0.01
    cType = "_6_DOF"
    rLegOffset = [0.,  -0.00018, -0.102]
    #rLegOffset[2] += 0.006
    rLegNormal = [0,0,1]
    rLegx = 0.1; rLegy = 0.06

    lLegOffset = [0.,  -0.00018, -0.102]
    #lLegOffset[2] += 0.006
    lLegNormal = [0,0,1]
    lLegx = 0.1; lLegy = 0.06

    rArmOffset = [-0.01,0.,-0.154]
    rArmNormal = [0,0,1]
    rArmx = 0.005; rArmy = 0.005

    lArmOffset = [-0.01,0.,-0.154]
    lArmNormal = [0,0,1]
    lArmx = 0.005; lArmy = 0.005

    kinematicConstraintsPath="package://talos-rbprm/com_inequalities/"
    rLegKinematicConstraints=kinematicConstraintsPath+rleg+"_com_constraints.obj"
    lLegKinematicConstraints=kinematicConstraintsPath+lleg+"_com_constraints.obj" 
    rArmKinematicConstraints=kinematicConstraintsPath+rarm+"_com_constraints.obj" 
    lArmKinematicConstraints=kinematicConstraintsPath+larm+"_com_constraints.obj"
    minDist=0.4
    # data used by scripts : 
    limbs_names = [rLegId,lLegId,rArmId,lArmId]
    dict_limb_rootJoint = {rLegId:rleg, lLegId:lleg, rArmId:rarm, lArmId:larm}
    dict_limb_joint = {rLegId:rfoot, lLegId:lfoot, rArmId:rhand, lArmId:lhand}
    dict_limb_color_traj = {rfoot:[0,1,0,1], lfoot:[1,0,0,1],rhand:[0,0,1,1],lhand:[0.9,0.5,0,1]}
    FOOT_SAFETY_SIZE = 0.03
    # size of the contact surface (x,y)
    dict_size={rfoot:[0.2 , 0.13], lfoot:[0.2 , 0.13],rhand:[0.1, 0.1],lhand:[0.1, 0.1]}


    # various offset used by scripts : 

    MRsole_offset = SE3.Identity()
    MRsole_offset.translation = np.matrix(rLegOffset).T
    MLsole_offset = SE3.Identity()
    MLsole_offset.translation = np.matrix(lLegOffset).T
    MRhand_offset = SE3.Identity()
    MRhand_offset.translation = np.matrix(rArmOffset).T
    MLhand_offset = SE3.Identity()
    MLhand_offset.translation = np.matrix(lArmOffset).T
    dict_offset = {rfoot:MRsole_offset, lfoot:MLsole_offset, rhand:MRhand_offset, lhand:MLhand_offset}
    dict_normal = {rfoot:rLegNormal, lfoot:lLegNormal, rhand:rArmNormal, lhand:lArmNormal}

    # display transform :

    #MRsole_display = MRsole_offset.copy()
    #MLsole_display = MLsole_offset.copy()
    MRsole_display = SE3.Identity()
    MLsole_display = SE3.Identity()
    MRhand_display = SE3.Identity()
    #MRhand_display.translation = np.matrix([0,  0., -0.11])
    MLhand_display = SE3.Identity()
    #MLhand_display.translation = np.matrix([0,  0., -0.11])
    dict_display_offset = {rfoot:MRsole_display, lfoot:MLsole_display, rhand:MRhand_display, lhand:MLhand_display}


    def __init__ (self, name = None,load = True):
        Parent.__init__ (self,load)
        if load:
            self.loadFullBodyModel(self.urdfName, self.rootJointType, self.meshPackageName, self.packageName, self.urdfSuffix, self.srdfSuffix)
        if name != None:
            self.name = name


    def loadAllLimbs(self,heuristic, analysis = None, nbSamples = nbSamples, octreeSize = octreeSize):
        for id in self.limbs_names:
            eff = self.dict_limb_joint[id]
            self.addLimb(id,self.dict_limb_rootJoint[id],eff,self.dict_offset[eff].translation.T.tolist()[0],self.dict_normal[eff],self.dict_size[eff][0]/2.,self.dict_size[eff][1]/2.,nbSamples,heuristic,octreeSize,self.cType,kinematicConstraintsPath=self.kinematicConstraintsPath+self.dict_limb_rootJoint[id]+"_com_constraints.obj",kinematicConstraintsMin=self.minDist)
            if analysis :
                self.runLimbSampleAnalysis(id, analysis, True)
        

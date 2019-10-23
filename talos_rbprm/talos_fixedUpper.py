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

import numpy as np
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody as Parent
from pinocchio import SE3


class Robot(Parent):
    #
    #  Information to retrieve urdf and srdf files.
    packageName = "talos_data"
    meshPackageName = "talos_data"
    rootJointType = "freeflyer"
    urdfName = "talos"
    urdfSuffix = "_reduced_fixedUpper_safeFeet"
    srdfSuffix = ""

    # Information about the names of thes joints defining the limbs of the robot
    rLegId = 'talos_rleg_rom'
    rleg = 'leg_right_1_joint'
    rfoot = 'leg_right_6_joint'

    lLegId = 'talos_lleg_rom'
    lleg = 'leg_left_1_joint'
    lfoot = 'leg_left_6_joint'

    referenceConfig = [
        0.0,
        0.0,
        1.01927,
        0.0,
        0.0,
        0.0,
        1.,  # Free flyer
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,  # Left Leg
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,  # Right Leg
    ]
    referenceConfig_legsApart = [
        0.0,
        0.0,
        1.01927,
        0.0,
        0.0,
        0.0,
        1.,  # Free flyer
        0.0,
        0.06,
        -0.411354,
        0.859395,
        -0.448041,
        -0.061708,  # Left Leg
        0.0,
        -0.06,
        -0.411354,
        0.859395,
        -0.448041,
        0.058292,  # Right Leg
    ]
    referenceConfig_armsFront = [
        0.0,
        0.0,
        1.01927,
        0.,
        0.0,
        0.0,
        1.,  # Free flyer
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,  # Left Leg
        0.0,
        0.0,
        -0.411354,
        0.859395,
        -0.448041,
        -0.001708,  # Right Leg
    ]
    referenceConfig_legsSide = [
        0.0,
        0.0,
        0.9832773,
        1,
        0.0,
        0.0,
        0.0,  # Free flyer
        1.57,
        0.0,
        -0.611354,
        1.059395,
        -0.448041,
        -0.001708,  # Left Leg
        -1.57,
        0.0,
        -0.611354,
        1.059395,
        -0.448041,
        -0.001708,  # Right Leg
    ]
    postureWeights = [
        0,
        0,
        0,
        0,
        0,
        0,  # freeflyer
        20.,
        100.,
        0.,
        0.1,
        0.,
        1.,  # lleg
        20.,
        100.,
        0.,
        0.1,
        0.,
        1.,  # rleg
    ]
    postureWeights_straff = [
        0,
        0,
        0,
        0,
        0,
        0,  # freeflyer
        100.,
        1.,
        10.,
        10,
        1.,
        0.,  # lleg
        100.,
        1.,
        10.,
        10,
        1.,
        0.,  # rleg
    ]

    # informations required to generate the limbs databases :
    nbSamples = 50000
    octreeSize = 0.01
    cType = "_6_DOF"
    rLegOffset = [0., -0.00018, -0.102]
    # rLegOffset[2] += 0.006
    rLegNormal = [0, 0, 1]
    rLegx = 0.1
    rLegy = 0.06

    lLegOffset = [0., -0.00018, -0.102]
    # lLegOffset[2] += 0.006
    lLegNormal = [0, 0, 1]
    lLegx = 0.1
    lLegy = 0.06

    kinematicConstraintsPath = "package://talos-rbprm/com_inequalities/"
    rLegKinematicConstraints = kinematicConstraintsPath + rleg + "_com_constraints.obj"
    lLegKinematicConstraints = kinematicConstraintsPath + lleg + "_com_constraints.obj"
    minDist = 0.4
    # data used by scripts :
    limbs_names = [rLegId, lLegId]
    dict_limb_rootJoint = {rLegId: rleg, lLegId: lleg}
    dict_limb_joint = {rLegId: rfoot, lLegId: lfoot}
    dict_limb_color_traj = {rfoot: [0, 1, 0, 1], lfoot: [1, 0, 0, 1]}
    FOOT_SAFETY_SIZE = 0.03
    # size of the contact surface (x,y)
    dict_size = {rfoot: [0.2, 0.13], lfoot: [0.2, 0.13]}

    # various offset used by scripts :

    MRsole_offset = SE3.Identity()
    MRsole_offset.translation = np.matrix(rLegOffset).T
    MLsole_offset = SE3.Identity()
    MLsole_offset.translation = np.matrix(lLegOffset).T
    dict_offset = {rfoot: MRsole_offset, lfoot: MLsole_offset}
    dict_normal = {rfoot: rLegNormal, lfoot: lLegNormal}

    # display transform :

    # MRsole_display = MRsole_offset.copy()
    # MLsole_display = MLsole_offset.copy()
    MRsole_display = SE3.Identity()
    MLsole_display = SE3.Identity()
    dict_display_offset = {rfoot: MRsole_display, lfoot: MLsole_display}

    kneeIds = {"Left": 10, "Right": 16}

    def __init__(self, name=None, load=True):
        Parent.__init__(self, load)
        if load:
            self.loadFullBodyModel(self.urdfName, self.rootJointType, self.meshPackageName, self.packageName,
                                   self.urdfSuffix, self.srdfSuffix)
        if name is not None:
            self.name = name
        self.joint1L_bounds_prev = self.getJointBounds('leg_left_1_joint')
        self.joint6L_bounds_prev = self.getJointBounds('leg_left_6_joint')
        self.joint2L_bounds_prev = self.getJointBounds('leg_left_2_joint')
        self.joint1R_bounds_prev = self.getJointBounds('leg_right_1_joint')
        self.joint6R_bounds_prev = self.getJointBounds('leg_right_6_joint')
        self.joint2R_bounds_prev = self.getJointBounds('leg_right_2_joint')

    def loadAllLimbs(self, heuristic, analysis=None, nbSamples=nbSamples, octreeSize=octreeSize):
        for id in self.limbs_names:
            eff = self.dict_limb_joint[id]
            self.addLimb(id,
                         self.dict_limb_rootJoint[id],
                         eff,
                         self.dict_offset[eff].translation.T.tolist()[0],
                         self.dict_normal[eff],
                         self.dict_size[eff][0] / 2.,
                         self.dict_size[eff][1] / 2.,
                         nbSamples,
                         heuristic,
                         octreeSize,
                         self.cType,
                         kinematicConstraintsPath=self.kinematicConstraintsPath + self.dict_limb_rootJoint[id] +
                         "_com_constraints.obj",
                         kinematicConstraintsMin=self.minDist)
            if analysis:
                self.runLimbSampleAnalysis(id, analysis, True)

    def setConstrainedJointsBounds(self):
        self.setJointBounds('leg_left_1_joint', [-0.34, 1.4])
        self.setJointBounds('leg_left_6_joint', [-0.25, 0.25])
        self.setJointBounds('leg_left_2_joint', [-0.25, 0.25])
        self.setJointBounds('leg_right_1_joint', [-1.4, 0.34])
        self.setJointBounds('leg_right_6_joint', [-0.25, 0.25])
        self.setJointBounds('leg_right_2_joint', [-0.25, 0.25])

    def resetJointsBounds(self):
        self.setJointBounds('leg_left_1_joint', self.joint1L_bounds_prev)
        self.setJointBounds('leg_left_6_joint', self.joint6L_bounds_prev)
        self.setJointBounds('leg_left_2_joint', self.joint2L_bounds_prev)
        self.setJointBounds('leg_right_1_joint', self.joint1R_bounds_prev)
        self.setJointBounds('leg_right_6_joint', self.joint6R_bounds_prev)
        self.setJointBounds('leg_right_2_joint', self.joint2R_bounds_prev)

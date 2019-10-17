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

from hpp.corbaserver.rbprm.rbprmbuilder import Builder as Parent

class Robot (Parent):
    ##
    #  Information to retrieve urdf and srdf files.
    rootJointType = 'freeflyer'
    packageName = 'talos-rbprm'
    meshPackageName = 'talos-rbprm'
    urdfName = 'talos_trunk'
    urdfNameRom =  ['talos_larm_rom','talos_rarm_rom','talos_lleg_rom','talos_rleg_rom']
    urdfSuffix = ""
    srdfSuffix = ""

    rLegId = 'talos_rleg_rom'
    lLegId = 'talos_lleg_rom'
    rArmId = 'talos_rarm_rom'
    lArmId = 'talos_larm_rom'

    legX = 0.1
    legY = 0.06

    ref_height = 1.02127
    # reference position of the end effector position for each ROM

    ref_EE_lLeg = [-0.008846952891378526, 0.0848172440888579,-1.019272022956703]
    ref_EE_lLeg[0]=0. # assure symetry of dynamic constraints on flat ground
    ref_EE_rLeg = [-0.008846952891378526,-0.0848172440888579,-1.019272022956703]
    ref_EE_rLeg[0] = 0.
    ref_EE_lArm = [0.13028765672452458, 0.44360498616312666,-0.2881211563246389]
    ref_EE_rArm = [0.13028765672452458,- 0.44360498616312666,-0.2881211563246389]


    def __init__ (self,name = None, load = True):
        Parent.__init__ (self,load)
        if load:
            self.loadModel(self.urdfName, self.urdfNameRom, self.rootJointType, self.meshPackageName, self.packageName, self.urdfSuffix, self.srdfSuffix)
        if name != None:
            self.name = name
        if 'talos_lleg_rom' in self.urdfNameRom:
            self.setReferenceEndEffector('talos_lleg_rom',self.ref_EE_lLeg)
        if 'talos_rleg_rom' in self.urdfNameRom:
            self.setReferenceEndEffector('talos_rleg_rom',self.ref_EE_rLeg)
        if 'talos_larm_rom' in self.urdfNameRom:
            self.setReferenceEndEffector('talos_larm_rom',self.ref_EE_lArm)
        if 'talos_rarm_rom' in self.urdfNameRom:
            self.setReferenceEndEffector('talos_rarm_rom',self.ref_EE_rArm)

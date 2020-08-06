#!/usr/bin/env python
# Copyright (c) 2019 CNRS
# Author: Pierre Fernbach

from hpp.corbaserver.rbprm.rbprmbuilder import Builder as Parent


class Robot(Parent):
    ##
    #  Information to retrieve urdf and srdf files.
    name = "talos_trunk"
    rootJointType = 'freeflyer'
    packageName = 'talos-rbprm'
    meshPackageName = 'talos-rbprm'
    urdfName = 'talos_trunk'
    urdfNameRom = ['talos_larm_rom', 'talos_rarm_rom', 'talos_lleg_rom', 'talos_rleg_rom']
    urdfSuffix = ""
    srdfSuffix = ""

    rLegId = 'talos_rleg_rom'
    lLegId = 'talos_lleg_rom'
    rArmId = 'talos_rarm_rom'
    lArmId = 'talos_larm_rom'

    legX = 0.1
    legY = 0.06

    ref_height = 1.0225
    # reference position of the end effector position for each ROM
    ref_EE_lLeg = [-0.008846952891378526, 0.0848172440888579, -1.01977]
    ref_EE_lLeg[0] = 0.  # assure symetry of dynamic constraints on flat ground
    ref_EE_rLeg = [-0.008846952891378526, -0.0848172440888579, -1.01977]
    ref_EE_rLeg[0] = 0.
    ref_EE_lArm = [0.13028765672452458, 0.44360498616312666, -0.2881211563246389]
    ref_EE_rArm = [0.13028765672452458, -0.44360498616312666, -0.2881211563246389]
    dict_ref_effector_from_root = {rLegId:ref_EE_rLeg, # Effector position in the reference configuration, in the root frame
                                   lLegId:ref_EE_lLeg,
                                   rArmId:ref_EE_rArm,
                                   lArmId:ref_EE_lArm}

    def __init__(self, name=None, load=True, client=None, clientRbprm=None):
        if name is not None:
            self.name = name
        Parent.__init__(self, self.name, self.rootJointType, load, client, None, clientRbprm)
        if 'talos_lleg_rom' in self.urdfNameRom:
            self.setReferenceEndEffector('talos_lleg_rom', self.ref_EE_lLeg)
        if 'talos_rleg_rom' in self.urdfNameRom:
            self.setReferenceEndEffector('talos_rleg_rom', self.ref_EE_rLeg)
        if 'talos_larm_rom' in self.urdfNameRom:
            self.setReferenceEndEffector('talos_larm_rom', self.ref_EE_lArm)
        if 'talos_rarm_rom' in self.urdfNameRom:
            self.setReferenceEndEffector('talos_rarm_rom', self.ref_EE_rArm)

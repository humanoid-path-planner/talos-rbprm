from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer


import quaternion as quat

packageName = "talos_description"
meshPackageName = "talos_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "talos"
urdfSuffix = "_full_v2"
srdfSuffix = ""

fullBody = FullBody ()
 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

nbSamples = 100000

ps = ProblemSolver( fullBody )

r = Viewer (ps)

rootName = 'base_joint_xyz'


q_0 = [ 0,0,0,1,0,0,0, # root_joint
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_left
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_right
        0, 0.006761, # torso
        0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, # arm_left
        0, 0, 0, 0, 0, 0, 0, # gripper_left
        -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, # arm_right
        0, 0, 0, 0, 0, 0, 0, # gripper_right
        0, 0 # head
        ]
r(q_0)


r.addLandmark(r.sceneName,1)
r.addLandmark('talos/gripper_left_inner_single_link',0.3)
r.addLandmark('talos/gripper_right_inner_single_link',0.3)
r.addLandmark('talos/left_sole_link',0.3)
r.addLandmark('talos/right_sole_link',0.3)


rLegId = 'rleg'
rLeg = 'leg_right_1_joint'
rfoot = 'leg_right_sole_fix_joint'
rLegOffset = [0,0,0.01]
rLegNormal = [0,0,1]
rLegx = 0.06; rLegy = 0.1
fullBody.addLimb(rLegId,rLeg,rfoot,rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, "EFORT", 0.01)

lLegId = 'lleg'
lLeg = 'leg_left_1_joint'
lfoot = 'leg_left_sole_fix_joint'
lLegOffset = [0,0,0.01]
lLegNormal = [0,0,1]
lLegx = 0.06; lLegy = 0.1
fullBody.addLimb(lLegId,lLeg,lfoot,lLegOffset,rLegNormal, lLegx, lLegy, nbSamples, "EFORT", 0.01)

rarmId = 'rarm'
rarm = 'arm_right_1_joint'
rHand = 'gripper_right_inner_single_joint'
rArmOffset = [0,0,0.1]
rArmNormal = [0,0,1]
rArmx = 0.02; rArmy = 0.02
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, nbSamples, "EFORT", 0.01)

larmId = 'larm'
larm = 'arm_left_1_joint'
lHand = 'gripper_left_inner_single_joint'
lArmOffset = [0,0,-0.1]
lArmNormal = [0,0,1]
lArmx = 0.02; lArmy = 0.02
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, nbSamples, "EFORT", 0.01)



def printEffPosition(limbId, nbSamples):
	limit = nbSamples-1;
	f1=open('./data/talos/roms/'+limbId+'.erom', 'w+')
	for i in range(0,limit):
		q = fullBody.getSamplePosition(limbId,i)
		f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
	f1.close()

printEffPosition(rarmId, nbSamples)
printEffPosition(rLegId, nbSamples)
printEffPosition(larmId, nbSamples)
printEffPosition(lLegId, nbSamples)

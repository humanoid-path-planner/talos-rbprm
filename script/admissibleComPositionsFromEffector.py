# Importing helper class for RBPRM
import numpy as np
import quaternion as quat
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody

packageName = "talos_description"
meshPackageName = "talos_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "talos"
urdfSuffix = "_full_v2"
srdfSuffix = ""

#  This time we load the full body model of HyQ
fullBody = FullBody()
fullBody.loadFullBodyModel(
    urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix
)
fullBody.setJointBounds("base_joint_xyz", [-20, 20, -20, 20, -20, 20])

rootName = "base_joint_xyz"
nbSamples = 100000

q_0 = [
    0,
    0,
    0,
    1,
    0,
    0,
    0,  # root_joint
    0.0,
    0.0,
    -0.411354,
    0.859395,
    -0.448041,
    -0.001708,  # leg_left
    0.0,
    0.0,
    -0.411354,
    0.859395,
    -0.448041,
    -0.001708,  # leg_right
    0,
    0.006761,  # torso
    0.25847,
    0.173046,
    -0.0002,
    -0.525366,
    0,
    0,
    0.1,  # arm_left
    0,
    0,
    0,
    0,
    0,
    0,
    0,  # gripper_left
    -0.25847,
    -0.173046,
    0.0002,
    -0.525366,
    0,
    0,
    0.1,  # arm_right
    0,
    0,
    0,
    0,
    0,
    0,
    0,  # gripper_right
    0,
    0,  # head
]
"""
r.addLandmark(r.sceneName,1)
r.addLandmark('talos/gripper_left_inner_single_link',0.3)
r.addLandmark('talos/gripper_right_inner_single_link',0.3)
r.addLandmark('talos/left_sole_link',0.3)
r.addLandmark('talos/right_sole_link',0.3)
"""

rLegId = "rleg"
rLeg = "leg_right_1_joint"
rfoot = "leg_right_sole_fix_joint"
rLegOffset = [0, 0, 0.01]
rLegNormal = [0, 0, 1]
rLegx = 0.1
rLegy = 0.06
fullBody.addLimb(
    rLegId, rLeg, rfoot, rLegOffset, rLegNormal, rLegx, rLegy, nbSamples, "EFORT", 0.01
)

lLegId = "lleg"
lLeg = "leg_left_1_joint"
lfoot = "leg_left_sole_fix_joint"
lLegOffset = [0, 0, 0.01]
lLegNormal = [0, 0, 1]
lLegx = 0.1
lLegy = 0.06
fullBody.addLimb(
    lLegId, lLeg, lfoot, lLegOffset, rLegNormal, lLegx, lLegy, nbSamples, "EFORT", 0.01
)

rarmId = "rarm"
rarm = "arm_right_1_joint"
rHand = "gripper_right_inner_single_joint"
rArmOffset = [0, 0, 0.1]
rArmNormal = [0, 0, 1]
rArmx = 0.02
rArmy = 0.02
fullBody.addLimb(
    rarmId, rarm, rHand, rArmOffset, rArmNormal, rArmx, rArmy, nbSamples, "EFORT", 0.01
)

larmId = "larm"
larm = "arm_left_1_joint"
lHand = "gripper_left_inner_single_joint"
lArmOffset = [0, 0, -0.1]
lArmNormal = [0, 0, 1]
lArmx = 0.02
lArmy = 0.02
fullBody.addLimb(
    larmId, larm, lHand, lArmOffset, lArmNormal, lArmx, lArmy, nbSamples, "EFORT", 0.01
)

zeroConf = [0, 0, 0, 1, 0, 0, 0]
q_0[0:7] = zeroConf

fullBody.setCurrentConfig(q_0)

effectors = [rfoot, lfoot, lHand, rHand]
limbIds = [rLegId, lLegId, larmId, rarmId]

# effectorName = rfoot
# limbId = rLegId
# q = fullBody.getSample(limbId, 1)
# fullBody.setCurrentConfig(q) #setConfiguration matching sample
# qEffector = fullBody.getJointPosition(effectorName)
# q0 = quat.Quaternion(qEffector[3:7])
# rot = q0.toRotationMatrix() #compute rotation matrix world -> local
# p = qEffector[0:3] #(0,0,0) coordinate expressed in effector fram
# rm=np.zeros((4,4))
# for i in range(0,3):
# for j in range(0,3):
# rm[i,j] = rot[i,j]
# for i in range(0,3):
# rm[i,3] = qEffector[i]
# rm[3,3] = 1
# invrm = np.linalg.inv(rm)
# p = invrm.dot([0,0,0,1])

points = [[], [], [], []]


def printComPosition(nbConfigs):
    num_invalid = 0
    for i in range(0, nbConfigs):
        q = fullBody.shootRandomConfig()
        q[0:7] = zeroConf
        fullBody.setCurrentConfig(q)  # setConfiguration matching sample
        com = fullBody.getCenterOfMass()
        for x in range(0, 3):
            q[x] = -com[x]
        fullBody.setCurrentConfig(q)
        # print ("final com" + str(com))
        # print ("final com" + str(fullBody.getCenterOfMass()))
        if fullBody.isConfigValid(q)[0]:
            for j in range(0, len(effectors)):
                effectorName = effectors[j]
                # limbId = limbIds[j]
                qEffector = fullBody.getJointPosition(effectorName)
                q0 = quat.Quaternion(qEffector[3:7])
                rot = q0.toRotationMatrix()  # compute rotation matrix world -> local
                p = qEffector[0:3]  # (0,0,0) coordinate expressed in effector fram
                rm = np.zeros((4, 4))
                for k in range(0, 3):
                    for j in range(0, 3):
                        rm[k, j] = rot[k, j]
                for m in range(0, 3):
                    rm[m, 3] = qEffector[m]
                rm[3, 3] = 1
                invrm = np.linalg.inv(rm)
                p = invrm.dot([0, 0, 0, 1])
                points[j].append(p)
                # print (points[j])
        else:
            num_invalid += 1
    for j in range(0, len(limbIds)):
        f1 = open("./" + str(limbIds[j]) + "_com.erom", "w+")
        for p in points[j]:
            f1.write(str(p[0]) + "," + str(p[1]) + "," + str(p[2]) + "\n")
        f1.close()
    print("%invalid ", (float)(num_invalid) / (float)(nbConfigs) * 100, "%")


# printRootPosition(rLegId, rfoot, nbSamples)
# printRootPosition(lLegId, lfoot, nbSamples)
# printRootPosition(rarmId, rHand, nbSamples)
# printRootPosition(larmId, lHand, nbSamples)
printComPosition(100000)

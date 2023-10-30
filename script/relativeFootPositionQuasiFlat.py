from __future__ import print_function

import numpy as np

# from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.rbprm import rbprmstate, state_alg
from hpp.corbaserver.rbprm.tools.constants_and_tools import hull_to_obj
from hpp.gepetto import ViewerFactory
from numpy import array
from numpy.linalg import norm
from pinocchio import Quaternion
from scipy.spatial import ConvexHull

from talos_rbprm.talos import Robot

NUM_SAMPLES = 18000
IT_DISPLAY_PROGRESS = NUM_SAMPLES / 10
MIN_DIST_BETWEEN_FEET_Y = 0.18
MAX_DIST_BETWEEN_FEET_X = 0.35
MIN_HEIGHT_COM = 0.3
# margin used to constrain the com y position :
# if it's on the left of the left foot or on the right of the right foot
# for more than this margin, we reject this sample:
MARGIN_FEET_SIDE = 0.05

fullBody = Robot()
# fullBody.setJointBounds ("base_joint_xyz", [-2,2, -2, 2, -2, 2])
fullBody.setJointBounds("root_joint", [-20, 20, -20, 20, -20, 20])
fullBody.setConstrainedJointsBounds()

nbSamples = 1

ps = ProblemSolver(fullBody)
vf = ViewerFactory(ps)
v = vf.createViewer()
rootName = "root_joint"

nbSamples = 10000
heuristic = "static"
print("gen limb db")
fullBody.addLimb(
    fullBody.rLegId,
    fullBody.rleg,
    fullBody.rfoot,
    fullBody.rLegOffset,
    fullBody.rLegNormal,
    fullBody.rLegx,
    fullBody.rLegy,
    nbSamples,
    heuristic,
    0.01,
    kinematicConstraintsPath=fullBody.rLegKinematicConstraints,
    kinematicConstraintsMin=0.75,
)
fullBody.runLimbSampleAnalysis(fullBody.rLegId, "ReferenceConfiguration", True)
fullBody.addLimb(
    fullBody.lLegId,
    fullBody.lleg,
    fullBody.lfoot,
    fullBody.lLegOffset,
    fullBody.rLegNormal,
    fullBody.lLegx,
    fullBody.lLegy,
    nbSamples,
    heuristic,
    0.01,
    kinematicConstraintsPath=fullBody.lLegKinematicConstraints,
    kinematicConstraintsMin=0.75,
)
fullBody.runLimbSampleAnalysis(fullBody.lLegId, "ReferenceConfiguration", True)
print("db generated.")
rLegId = fullBody.rLegId
lLegId = fullBody.lLegId
rfoot = fullBody.rfoot
lfoot = fullBody.lfoot
rLegOffset = fullBody.rLegOffset
lLegOffset = fullBody.lLegOffset

# make sure this is 0
q_0 = fullBody.getCurrentConfig()
zeroConf = [0, 0, 0, 0, 0, 0, 1.0]
q_0[0:7] = zeroConf
fullBody.setCurrentConfig(q_0)

effectors = [rfoot, lfoot]
limbIds = [rLegId, lLegId]
offsets = [array(rLegOffset), array(lLegOffset)]

points = [[], []]
# compoints = [[[0.012471792486262121, 0.0015769611415203033, 0.8127583093263778]],
# [[0.012471792486262121, 0.0015769611415203033, 0.8127583093263778]]]
compoints = [[], []]

success = 0
fails = 0


def genFlat():
    q = fullBody.shootRandomConfig()
    q[0:7] = zeroConf
    fullBody.setCurrentConfig(q)
    # v(q)
    posrf = fullBody.getJointPosition(rfoot)[:3]
    poslf = fullBody.getJointPosition(lfoot)[:3]
    s = rbprmstate.State(fullBody, q=q, limbsIncontact=limbIds)
    s, succ = state_alg.addNewContact(
        s, rLegId, posrf, [0.0, 0.0, 1.0], num_max_sample=0
    )
    if succ:
        s, succ = state_alg.addNewContact(
            s, lLegId, poslf, [0.0, 0.0, 1.0], num_max_sample=0
        )
    if succ:
        succ = (
            fullBody.isConfigValid(q)[0]
            and norm(array(posrf[:2]) - array(poslf[:2])) >= 0.3
        )
    # print("sid = ", s.sId)
    # if succ and norm (array(posrf[:2]) - array(poslf[:2]) ) <= 0.1:
    if succ and norm(array(posrf) - array(poslf)) <= 0.1:
        v(s.q())
    return s.q(), succ, s, [posrf, poslf]


def printFootPositionRelativeToOther(nbConfigs):
    for i in range(0, nbConfigs):
        if i > 0 and not i % IT_DISPLAY_PROGRESS:
            print(int((i * 100) / nbConfigs), " % done")
        q, succ, s, pos = genFlat()
        if succ:
            global success
            success += 1
            addCom = True
            for j in range(0, len(effectors)):
                # for j in range(1):
                fullBody.setCurrentConfig(q)
                otheridx = (j + 1) % 2
                # print("otheridx", otheridx)
                # print("q ", q[:3])
                oeffectorName = effectors[otheridx]
                # oqEffector = fullBody.getJointPosition(oeffectorName)
                pos_other = fullBody.getJointPosition(oeffectorName)
                # print("other pos 1", pos_other[:3])

                effectorName = effectors[j]
                # limbId = limbIds[j]
                qEffector = fullBody.getJointPosition(effectorName)
                # print("pos 1", qEffector[:3])

                qtr = q[:]
                qtr[:3] = [
                    qtr[0] - pos_other[0],
                    qtr[1] - pos_other[1],
                    qtr[2] - pos_other[2],
                ]
                # for l in range(3):
                # qtr[l] -= pos_other[l]
                # qtr[i] -= qEffector[i]
                fullBody.setCurrentConfig(qtr)

                effectorName = effectors[j]
                # limbId = limbIds[j]
                qEffector = fullBody.getJointPosition(effectorName)
                # print("pos 2", qEffector[3:])
                # print("pos 2", qEffector[:3])

                # print("other effectorName 1", oeffectorName)
                # print("effectorName 1", effectorName)
                pos_other = fullBody.getJointPosition(oeffectorName)
                # check current joint pos is now zero
                # print("other pos 2", pos_other[:3])
                # v(qtr)
                q0 = Quaternion(qEffector[6], qEffector[3], qEffector[4], qEffector[5])
                rot = q0.matrix()  # compute rotation matrix world -> local
                p = qEffector[0:3]  # (0,0,0) coordinate expressed in effector fram
                rm = np.zeros((4, 4))
                for k in range(0, 3):
                    for j in range(0, 3):
                        rm[k, j] = rot[k, j]
                for m in range(0, 3):
                    rm[m, 3] = qEffector[m]
                rm[3, 3] = 1
                invrm = np.linalg.inv(rm)
                # print(invrm)
                # p = invrm.dot([0,0,0,1])
                p = invrm.dot([0, 0, 0.0, 1])
                # print("p ", p)
                # print(norm (array(posrf) - array(poslf) ))
                if (
                    j == 0
                    and p[1] > MIN_DIST_BETWEEN_FEET_Y
                    and abs(p[0]) < MAX_DIST_BETWEEN_FEET_X
                ):
                    points[j].append(p[:3])
                elif (
                    j == 1
                    and p[1] < -MIN_DIST_BETWEEN_FEET_Y
                    and abs(p[0]) < MAX_DIST_BETWEEN_FEET_X
                ):
                    points[j].append(p[:3])
                else:
                    addCom = False
            # print(points[j])
            # now compute coms

            fullBody.setCurrentConfig(q)
            com = fullBody.getCenterOfMass()
            for x in range(0, 3):
                q[x] = -com[x]
            for j in range(0, len(effectors)):
                effectorName = effectors[j]
                # limbId = limbIds[j]
                qEffector = fullBody.getJointPosition(effectorName)
                q0 = Quaternion(qEffector[6], qEffector[3], qEffector[4], qEffector[5])
                rot = q0.matrix()  # compute rotation matrix world -> local
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
                # add offset
                rp = array(p[:3] - offsets[j]).tolist()
                # print("p ", p)
                # print("rp ")

                if rp[2] < MIN_HEIGHT_COM:
                    addCom = False
                if addCom:
                    if j == 1:
                        if rp[1] < MARGIN_FEET_SIDE:
                            compoints[j].append(rp)
                    else:
                        if rp[1] > -MARGIN_FEET_SIDE:
                            compoints[j].append(rp)

            # compoints[j].append(p[:3])

        else:
            global fails
            fails += 1
            # print(fullBody.isConfigValid(q)[1])
    # for j in range(0,len(limbIds)):
    # f1=open('./'+str(limbIds[j])+'_com.erom', 'w+')
    # for p in points[j]:
    # f1.write(str(p[0]) + "," + str(p[1]) + "," + str(p[2]) + "\n")
    # f1.close()


# printRootPosition(rLegId, rfoot, nbSamples)
# printRootPosition(lLegId, lfoot, nbSamples)
# printRootPosition(rarmId, rHand, nbSamples)
# printRootPosition(larmId, lHand, nbSamples)
printFootPositionRelativeToOther(NUM_SAMPLES)
print("successes ", success)
print("fails  ", fails)

hcomRF = ConvexHull(compoints[0])
hcomLF = ConvexHull(compoints[1])
hull_to_obj(hcomRF, compoints[0], "talos_COM_constraints_in_RF_effector_frame.obj")
hull_to_obj(hcomLF, compoints[1], "talos_COM_constraints_in_LF_effector_frame.obj")

hptsRF = ConvexHull(points[0])
hptsLF = ConvexHull(points[1])
hull_to_obj(hptsRF, points[0], "talos_LF_constraints_in_RF.obj")
hull_to_obj(hptsLF, points[1], "talos_RF_constraints_in_LF.obj")

# for k in range(2):
# hcom = ConvexHull(compoints[k])
# plot_hull(hcom, compoints[k], array(compoints[k]))

# hpts = ConvexHull(points[k])
# plot_hull(hpts, points[k], array(points[k]), color = "b", plot = k == 1 and True)

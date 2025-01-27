import pybullet as p
import numpy as np
from scipy import linalg as la
import time
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

# dt = 1/240
# maxTime = 10
# logTime = np.arange(0.0, maxTime, dt)

# creating environment and defining the robot
physicsClient = p.connect(p.GUI)
p.setGravity(0,0,-10)
c1 = p.loadURDF("./models/cube1.urdf.xml", useFixedBase=True)
c2 = p.loadURDF("./models/cube2.urdf.xml", useFixedBase=True)
boxId = p.loadURDF("./models/robot.urdf.xml", useFixedBase=True)

jointIndices = [1,2,3,4,5,6]
eefLinkIdx = 7

# defining key positions
startPos = [-np.pi/2,0,np.pi/2, 0, np.pi/2, 0]
cartesianPositions = [[1, 1, 1], [1, 1, 0.3], [1, 1, 1], [1, -1, 1], [1, -1, 0.3], [1, -1, 1]]
jointPositions = [startPos]
for xyzPos in cartesianPositions:
    temp = p.calculateInverseKinematics(boxId, eefLinkIdx, xyzPos)
    jointPositions.append(list(temp))
jointPositions.append(startPos)

# checking key positions
# for jPose in jointPositions:
#     p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jointIndices, targetPositions=jPose, controlMode=p.POSITION_CONTROL)
#     for _ in range(1000):
#         p.stepSimulation()
#         time.sleep(0.001)

pPoints = [[[0], [0.8], [1]],
            [[1], [1], [1]],
            [[1], [1], [0.3]],
            [[1], [1], [1]],
            [[1], [-1], [1]],
            [[1], [-1], [0.3]],
            [[1], [-1], [1]],
            [[0], [0.8], [1]]]
RPoints = [[[0, -1, 0],
            [-1, 0, 0],
            [0, 0, -1]],
            [[0.707, -0.707, 0],
            [-0.707, -0.707, 0],
            [0, 0, -1]],
            [[0.707, -0.707, 0],
            [-0.707, -0.707, 0],
            [0, 0, -1]],
            [[0.707, -0.707, 0],
            [-0.707, -0.707, 0],
            [0, 0, -1]],
            [[0.707, -0.707, 0],
            [-0.707, -0.707, 0],
            [0, 0, -1]],
            [[0.707, -0.707, 0],
            [-0.707, -0.707, 0],
            [0, 0, -1]],
            [[0.707, -0.707, 0],
            [-0.707, -0.707, 0],
            [0, 0, -1]],
            [[0, -1, 0],
            [-1, 0, 0],
            [0, 0, -1]]]

### Straight-Line Path for THETHA
def StraightLinePath(startPose, endPose, step):
    n = int(len(startPose))
    sLog = np.arange(0, 1+step, step)
    numSteps = int(len(sLog))

    posLog = np.zeros((n, numSteps))
    for k in range(numSteps):
        s = sLog[k]
        for th in range(n):
            posLog[th][k] = startPose[th] + s*(endPose[th] - startPose[th])

    return posLog
def TwoJointPointsStraightLinePath(th_start, th_end, step):
    jointStraightPath = StraightLinePath(th_start, th_end, step)

    sLog = np.arange(0, 1+step, step)
    # simulation
    for s in range(len(sLog)):
        # для каждого момента s получаем значения углов joint'в
        jPose = []
        for idx in range(len(jointIndices)):
            jPose.append(jointStraightPath[idx][s])
        p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jointIndices, targetPositions=jPose, controlMode=p.POSITION_CONTROL)
        p.stepSimulation()
        time.sleep(step*0.1)
    return jointStraightPath
def FullJointStraightPath(trajectoryKeyPoints):

    numPoints = len(trajectoryKeyPoints)

    jointStraightPath = []
    for th in range(len(jointIndices)):
        jointStraightPath.append([])

    for idx in range(numPoints-1):
        pieceOfPath = TwoJointPointsStraightLinePath(jointPositions[idx], jointPositions[idx+1], step=0.001) # 6 x 1001
        for i in range(len(jointIndices)):
            for th in pieceOfPath[i]:
                jointStraightPath[i].append(th)

    numSteps = len(jointStraightPath[0])
    sLog = np.arange(0, 1, 1/numSteps)

    plt.title("Joint Angles Trajectories")
    for th in range(len(jointIndices)):
        plt.subplot(3,3,th+1)
        plt.grid(True)
        plt.plot(sLog, jointStraightPath[th])
    plt.show()
# FullJointStraightPath(jointPositions)
###

### Screw Path
def Screwing(Xstart, Xend, step):
    Xstart = np.array(Xstart)
    Xend = np.array(Xend)

    sLog = np.arange(0, 1+step, step)
    XsLog = []
    for s in sLog:
        temp = Xstart @ la.expm(la.logm(la.inv(Xstart) @ Xend)*s)
        XsLog.append(temp)
    return XsLog
def ScrewPointToPointPath(Xstart, Xend, step):
    XsLog = Screwing(Xstart, Xend, step)
    # нужно вычленить матрицы вращения 3x3 и вектор координат 3x1
    RLog = []
    pLog = []
    thLog = []
    for x in XsLog:
        x = np.array(x)
        R = x[0:3, 0:3]
        p_ = x[0:3, 3]
        # преобразуем матрицу вращений в кватернион
        temp = Rotation.from_matrix(R)
        r = temp.as_quat()[[0, 1, 3, 2]]
        # найдем положениe THETHA
        th = p.calculateInverseKinematics(bodyUniqueId=boxId,
                                        endEffectorLinkIndex=eefLinkIdx,
                                        targetPosition=p_,
                                        targetOrientation=r)
        thLog.append(th)
        RLog.append(R)
        pLog.append(p_)

    # simulation
    for th in thLog:
        p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jointIndices, targetPositions=th, controlMode=p.POSITION_CONTROL)
        p.stepSimulation()
        time.sleep(step*10)
        
    return (pLog, RLog, thLog)

def FullScrewPath(RPoints, pPoints):
    for idx in range(len(pPoints)-1):
        R0 = RPoints[idx]
        R1 = RPoints[idx+1]
        p0 = pPoints[idx]
        p1 = pPoints[idx+1]
        Xstart = np.vstack([np.hstack([R0, p0]), [0, 0, 0, 1]])
        Xend = np.vstack([np.hstack([R1, p1]), [0, 0, 0, 1]])
        ScrewPointToPointPath(Xstart, Xend, step=0.001)
    return
# FullScrewPath()
###

### Decoupled Path
def FullDecoupledPath(RPoints, pPoints):
    for idx in range(len(pPoints)-1):
        R0 = RPoints[idx]
        R1 = RPoints[idx+1]
        p0 = pPoints[idx]
        p1 = pPoints[idx+1]

        pLog = np.array(StraightLinePath(p0, p1, step=0.01)).T
        print(pLog[0])

        RLog = Screwing(R0, R1, step=0.01)
        print(RLog[0])

        thLog = []

        for i in range(len(pLog)):
            # преобразуем матрицу вращений в кватернион
            temp = Rotation.from_matrix(RLog[i])
            r = temp.as_quat()[[0, 1, 3, 2]]

            # найдем положениe THETHA
            th = p.calculateInverseKinematics(bodyUniqueId=boxId,
                                            endEffectorLinkIndex=eefLinkIdx,
                                            targetPosition=pLog[i],
                                            targetOrientation=r)
            thLog.append(th)
        #             # simulation
        for th in thLog:
            p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jointIndices, targetPositions=th, controlMode=p.POSITION_CONTROL)
            p.stepSimulation()
            time.sleep(0.01)

    return
FullCartesianPositions =[[0, 0.8, 1], [1, 1, 1], [1, 1, 0.3], [1, 1, 1], [1, -1, 1], [1, -1, 0.3], [1, -1, 1], [0, 0.8, 1]]
#FullDecoupledPath(RPoints, FullCartesianPositions)
###

### Time-Scaling

p.disconnect()
import pybullet as p
import pybullet_data
import os
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

plane = p.loadURDF(("plane.urdf"), 0, 0, 2)
pinPos = [0, 0, 5]
pin = p.loadURDF("cube.urdf", pinPos, p.getQuaternionFromEuler([0, 0, 0]))

sphere_address = "/home/rishabh/Documents/GitHub/Robotics-Camp-2021/Basics of pyBullet/Week 2/Subpart 3/sphere.urdf"


bob1 = p.loadURDF(sphere_address, [-0.1, 0, 2.5],
                  p.getQuaternionFromEuler([0, 0, 0]))
bob2 = p.loadURDF(sphere_address, [0.1, 0, 2.5],
                  p.getQuaternionFromEuler([0, 0, 0]))
bob3 = p.loadURDF(sphere_address, [0.2, 0, 2.5],
                  p.getQuaternionFromEuler([0, 0, 0]))
bob4 = p.loadURDF(sphere_address, [0, 0, 2.5],
                  p.getQuaternionFromEuler([0, 0, 0]))
bob5 = p.loadURDF(sphere_address, [-0.2, 0, 2.5],
                  p.getQuaternionFromEuler([0, 0, 0]))

bobs = [bob1, bob2, bob3, bob4, bob5]

print("All Objects Loaded")


# time.sleep(5)
con_ID_1 = p.createConstraint(
    pin, -1, bob1, -1, p.JOINT_POINT2POINT, [0, 0, 0],       [-0.1, 0, -0.5],  [0, 0, 2])
con_ID_2 = p.createConstraint(
    pin, -1, bob2, -1, p.JOINT_POINT2POINT, [0, 0, 0],       [0.1, 0, -0.5],   [0, 0, 2])
con_ID_3 = p.createConstraint(pin, -1, bob3, -1, p.JOINT_POINT2POINT,
                              [0, 0, 0],       [0.2, 0, -0.5],    [0, 0, 2])  # extreme right
con_ID_4 = p.createConstraint(
    pin, -1, bob4, -1, p.JOINT_POINT2POINT, [0, 0, 0],       [0, 0, -0.5],      [0, 0, 2])
con_ID_5 = p.createConstraint(pin, -1, bob5, -1, p.JOINT_POINT2POINT,
                              [0, 0, 0],       [-0.2, 0, -0.5],   [0, 0, 2])  # extreme left


p.stepSimulation()

# print(p.getConstraintInfo(con_ID_1))
# print(p.getConstraintInfo(con_ID_2))
# print(p.getConstraintInfo(con_ID_3))
# print(p.getConstraintInfo(con_ID_4))
# print(p.getConstraintInfo(con_ID_5))

print("All Constraints Created")
print("Initializing Positions...")

for i in range(10000):
    p.resetBasePositionAndOrientation(
        pin, pinPos, p.getQuaternionFromEuler([0, 0, 0]))
    p.stepSimulation()

print("About to start the Simulation...")
# time.sleep(5)
# print(p.getPhysicsEngineParameters())

c = 0
while(True):

    # restitution = 1
    # restitutionVelocityThreshold = p.readUserDebugParameter(
    #     restitutionVelocityThresholdId)
    # p.setPhysicsEngineParameter(collisionFilterMode=0)

    # lateralFriction = 0
    # spinningFriction = 0
    # rollingFriction = 0

    # p.changeDynamics(plane, -1, lateralFriction=1)
    # p.changeDynamics(plane, -1, restitution=restitution)

    # for sphere in bobs:
    #     p.changeDynamics(sphere, -1, lateralFriction=lateralFriction)
    #     p.changeDynamics(sphere, -1, spinningFriction=spinningFriction)
    #     p.changeDynamics(sphere, -1, rollingFriction=rollingFriction)

    p.changeDynamics(bob1, -1, restitution=1,
                     linearDamping=0, angularDamping=0)
    p.changeDynamics(bob2, -1, restitution=1,
                     linearDamping=0, angularDamping=0)
    p.changeDynamics(bob3, -1, restitution=1,
                     linearDamping=0, angularDamping=0)
    p.changeDynamics(bob4, -1, restitution=1,
                     linearDamping=0, angularDamping=0)
    p.changeDynamics(bob5, -1, restitution=1,
                     linearDamping=0, angularDamping=0)

    c += 1
    p.resetBasePositionAndOrientation(
        pin, pinPos, p.getQuaternionFromEuler([0, 0, 0]))
    # print(c)

    if c <= 10000:   # just to make sure bobs are freely suspended
        # p.resetBasePositionAndOrientation(bob5,[-1,0,3.5],p.getQuaternionFromEuler([0,0,0]))
        p.stepSimulation()
        continue
    if c == 10005:
        p.applyExternalForce(bob5, -1, [-100, 0, 0], [0, 0, 0], p.LINK_FRAME)
        time.sleep(1)
        continue

    p.stepSimulation()
    time.sleep(1/240)

    # print("-"*100)
    # print("-"*100)
    # print(p.getBasePositionAndOrientation(bob1))
    # print(p.getBasePositionAndOrientation(bob2))
    # print(p.getBasePositionAndOrientation(bob3))
    # print(p.getBasePositionAndOrientation(bob4))
    # print(p.getBasePositionAndOrientation(bob5))

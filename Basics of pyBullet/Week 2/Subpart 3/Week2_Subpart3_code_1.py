import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)  # or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
carpos = [0, 0, 0.1]

car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)
for joint in range(numJoints):
    print(p.getJointInfo(car, joint))

targetVel = 10  # rad/s
maxForce = 100  # Newton
# p.applyExternalForce(car,3,[100,0,0],)
vel = 3
while (1):
    keys = p.getKeyboardEvents()
    for k, v in keys.items():

        ################################## forward ##################################
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = vel
            for joint in range(2, 6):
                p.setJointMotorControl2(
                    car, joint, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)

            p.stepSimulation()

        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(
                    car, joint, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)

            p.stepSimulation()

        ################################## backward ##################################
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = -vel
            for joint in range(2, 6):
                p.setJointMotorControl2(
                    car, joint, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)

            p.stepSimulation()

        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(
                    car, joint, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)

            p.stepSimulation()

        ################################## left ##################################
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = vel/2
            p.setJointMotorControl2(
                car, 2, p.VELOCITY_CONTROL, targetVelocity=targetVel/10, force=maxForce)
            p.setJointMotorControl2(
                car, 4, p.VELOCITY_CONTROL, targetVelocity=targetVel/10, force=maxForce)
            p.setJointMotorControl2(
                car, 3, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
            p.setJointMotorControl2(
                car, 5, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)

            p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            p.setJointMotorControl2(
                car, 2, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
            p.setJointMotorControl2(
                car, 4, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
            p.setJointMotorControl2(
                car, 3, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
            p.setJointMotorControl2(
                car, 5, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)

            p.stepSimulation()

        ################################## right ##################################
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = vel/2
            p.setJointMotorControl2(
                car, 2, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
            p.setJointMotorControl2(
                car, 4, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
            p.setJointMotorControl2(
                car, 3, p.VELOCITY_CONTROL, targetVelocity=targetVel/10, force=maxForce)
            p.setJointMotorControl2(
                car, 5, p.VELOCITY_CONTROL, targetVelocity=targetVel/10, force=maxForce)

            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            p.setJointMotorControl2(
                car, 2, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
            p.setJointMotorControl2(
                car, 4, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
            p.setJointMotorControl2(
                car, 3, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)
            p.setJointMotorControl2(
                car, 5, p.VELOCITY_CONTROL, targetVelocity=targetVel, force=maxForce)

            p.stepSimulation()

        ################################## rotation ##################################
        if (k == ord('r') and (v & p.KEY_IS_DOWN)):
            pos, orn = p.getBasePositionAndOrientation(car)
            orn = p.getEulerFromQuaternion(orn)
            p.resetBasePositionAndOrientation(
                car, pos, p.getQuaternionFromEuler([orn[0], orn[1], orn[2]+0.1]))
            time.sleep(0.2)
            # print("Rotation")
            p.stepSimulation()

        ################################## gear up ##################################
        if (k == ord('a') and (v & p.KEY_IS_DOWN)):
            vel += 1
            time.sleep(0.3)
            print("Geared up", end=" -> ")
            print("Forward and Backward speed now = ", vel)

        ################################## gear down ##################################
        if (k == ord('z') and (v & p.KEY_IS_DOWN)):
            vel -= 1
            time.sleep(0.3)
            print("Geared down", end=" -> ")
            print("Forward and Backward speed now = ", vel)


p.getContactPoints(car)

p.disconnect()

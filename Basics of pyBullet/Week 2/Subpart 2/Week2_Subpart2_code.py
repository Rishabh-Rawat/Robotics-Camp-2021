import pybullet as p
import pybullet_data
import time


def print_joint_details(body_ID):
    '''this function displays the details of all the joints of any body'''
    no_of_joints = p.getNumJoints(body_ID)
    print("\n\n\n")
    print("Number of joints = ", no_of_joints, end="\n\n")
    for i in range(no_of_joints):
        print("Joint ID = ", i)
        print(p.getJointInfo(body_ID, i))
        print(p.getJointState(body_ID, i))
        print("\n\n")
    print("\n")


# function to be filled to implement torque control
def Torque_control(body_ID):
    # find this value to climb the ramp without sliping and topling
    optimal_torque_value = -250

    '''
	this function should have the setJointMotorControl in TORQUE_CONTROL configuration
    with forc = optimal_force_value
    '''

    p.setJointMotorControl2(bodyUniqueId=body_ID, jointIndex=2,
                            controlMode=p.TORQUE_CONTROL, force=optimal_torque_value)
    p.setJointMotorControl2(bodyUniqueId=body_ID, jointIndex=3,
                            controlMode=p.TORQUE_CONTROL, force=optimal_torque_value)
    p.setJointMotorControl2(bodyUniqueId=body_ID, jointIndex=4,
                            controlMode=p.TORQUE_CONTROL, force=optimal_torque_value)
    p.setJointMotorControl2(bodyUniqueId=body_ID, jointIndex=5,
                            controlMode=p.TORQUE_CONTROL, force=optimal_torque_value)


# function to be filled to implement velocity control
def Velocity_control(body_ID):
    # Keep a constant non zero value for maxForce and try getting the velocity that makes it climb the ramp.
    maxForce = 100

    # find this value to climb the ramp without sliping
    optimal_velocity_value = -38
    '''
	this function should have the setJointMotorControl in VELOCITY_CONTROL configuration
	with targetvelocity = optimal_velocity_value 
	'''

    p.setJointMotorControl2(bodyUniqueId=body_ID, jointIndex=2, controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=optimal_velocity_value, force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=body_ID, jointIndex=3, controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=optimal_velocity_value, force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=body_ID, jointIndex=4, controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=optimal_velocity_value, force=maxForce)
    p.setJointMotorControl2(bodyUniqueId=body_ID, jointIndex=5, controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=optimal_velocity_value, force=maxForce)


if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    # these are the pre required conditions for the task.
    ramp = p.loadURDF("wedge.urdf")
    p.setGravity(0, 0, -10)
    p.changeDynamics(ramp, -1, lateralFriction=0.5)

    huskypos = [2, 0, 0.1]
    husky = p.loadURDF("husky/husky.urdf",
                       huskypos[0], huskypos[1], huskypos[2])

    '''
	1.print Number of Joints and joint info and state

	2.Get the user input about the control function they 
	want to simulate and see.(its upto you, it can be a string / int anything but just leave
	a comment with the legend to your menu)

	'''

    print_joint_details(husky)

    _link_name_to_index = {p.getBodyInfo(husky)[0].decode('UTF-8'): -1, }

    for _id in range(p.getNumJoints(husky)):
        _name = p.getJointInfo(husky, _id)[12].decode('UTF-8')
        _link_name_to_index[_name] = _id

    print(_link_name_to_index, end="\n\n")

    while(True):
        choice = input(
            "Enter 0 for Toruqe_control function \nEnter 1 for Velocity_control function\nYour Choice : ")
        if choice == "0":
            print("Torque Control Selected")
            break
        elif choice == "1":
            print("Velocity Control Selected")
            break
        else:
            print("Invalid value")

    i = 1
    while (1):
        time.sleep(.01)
        '''
		1.Here call either the Torque_control function or Velocity_control 
		function according to the initial user choice and apply the optimal velocity/Torque
		to the motors that you have found by experimentation.

		2.print base state and velocity 100 iteration steps once.
		'''
        if choice == "0":
            Torque_control(husky)
        else:
            Velocity_control(husky)

        if i == 100:
            cubePos, cubeOrn = p.getBasePositionAndOrientation(husky)
            print("\nBase Position : ", cubePos, cubeOrn)
            print("\nBase Link State Information : ",
                  p.getLinkState(bodyUniqueId=husky, linkIndex=0))
            print("\nBase Veclocity : ", p.getBaseVelocity(husky), end="\n\n")
            print("-"*100)
            i = 1
        else:
            i += 1

        p.stepSimulation()

    p.disconnect()

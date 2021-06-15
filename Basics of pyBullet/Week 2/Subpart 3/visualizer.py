import pybullet as p
import pybullet_data
import os

p.connect(p.GUI)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF("/home/rishabh/Documents/GitHub/Robotics-Camp-2021/Basics of pyBullet/Week 2/Subpart 3/sphere.urdf",[0, 0, 0], [0, 0, 0, 0.707])

# p.setJointMotorControl2(robot,1,p.VELOCITY_CONTROL,targetVelocity=0.01)

while(True):
	p.stepSimulation()

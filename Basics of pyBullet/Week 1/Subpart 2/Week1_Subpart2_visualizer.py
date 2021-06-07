
import pybullet as p
import pybullet_data
import os
'''
urdf file in the same folder as that of the python script
'''
file_path = os.getcwd()
file_name = "sample.urdf"
'''
these comands are explained in detail in the next subpart
for now u can directly use it to visualize the model
'''
p.connect(p.GUI)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF("/home/rishabh/Documents/GitHub/Robotics-Camp-2021/Basics of pyBullet/Week 1/Subpart 2/Week1_Subpart2.urdf")
p.resetBasePositionAndOrientation(robot, [0, 0, 0], [0, 0, 0, 0.707])
p.setGravity(0,0,0)

while(True):
	p.stepSimulation()

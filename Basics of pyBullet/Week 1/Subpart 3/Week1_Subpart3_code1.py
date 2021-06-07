import pybullet as p
import time
import pybullet_data
from math import *

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

planeId = p.loadURDF("plane.urdf")

boxId = p.loadURDF("/home/rishabh/Documents/GitHub/Robotics-Camp-2021/Basics of pyBullet/Week 1/Subpart 3/dabba.urdf",[0,0,1])
sampleId = p.loadURDF("/home/rishabh/Documents/GitHub/Robotics-Camp-2021/Basics of pyBullet/Week 1/Subpart 2/sample.urdf", [2,2,1])

while True:
    x = y = 0
    while(x**2+y**2 <= 9.8**2):
        print(sqrt(x**2+y**2+0))
        p.setGravity(x, y , 0)
        p.stepSimulation()
        time.sleep(1/240)

        x += 0.02 
        y += 0.02

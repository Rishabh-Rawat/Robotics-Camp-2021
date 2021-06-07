import pybullet as p
import time
import pybullet_data

def deploy(n):
    if n==0:
        return 0
    for i in range(n):
        dropID = p.loadURDF("/home/rishabh/Documents/GitHub/Robotics-Camp-2021/Basics of pyBullet/Week 1/Subpart 3/sphere.urdf",[i,0,5])
    return dropID

def make_fibo(l):
    fibo=[0,1]
    a=0
    b=1
    while(len(fibo)!=l):
        fibo.append(fibo[a]+fibo[b])
        a+=1
        b+=1
    return fibo

i=int(input("Enter the number of waves : "))

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)

time.sleep(5)

fibo = make_fibo(i)
c=0


for i in range(i):
    c+=1
    dropID = deploy(fibo[i])
    if dropID:
        while p.getBasePositionAndOrientation(dropID)[0][2]>0.15:
            p.stepSimulation()        
            time.sleep(1/10000)

print("All waves compleyted")

time.sleep(5)

p.disconnect()
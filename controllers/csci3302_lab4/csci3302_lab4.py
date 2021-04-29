"""csci3302_lab4 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import time
import random
import copy
import numpy as np
from controller import Robot, Motor, DistanceSensor

state = "line_follower" # Change this to anything else to stay in place to test coordinate transform functions

LIDAR_SENSOR_MAX_RANGE = 3 # Meters
LIDAR_ANGLE_BINS = 21 # 21 Bins to cover the angular range of the lidar, centered at 10
LIDAR_ANGLE_RANGE = 1.5708 # 90 degrees, 1.5708 radians

# These are your pose values that you will update by solving the odometry equations
pose_x = 0.197
pose_y = 0.678
pose_theta = 0 

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
MAX_SPEED = 6.28

# create the Robot instance.
robot=Robot()

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
	gs.enable(SIM_TIMESTEP)

# Initialize the Display    
display = robot.getDevice("display")

# get and enable lidar 
lidar = robot.getDevice("LDS-01")
lidar.enable(SIM_TIMESTEP)
lidar.enablePointCloud()

##### DO NOT MODIFY ANY CODE ABOVE THIS #####

##### Part 1: Setup Data structures
#
# Create an empty list for your lidar sensor readings here,
# as well as an array that contains the angles of each ray 
# in radians. The total field of view is LIDAR_ANGLE_RANGE,
# and there are LIDAR_ANGLE_BINS. An easy way to generate the
# array that contains all the angles is to use linspace from
# the numpy package.

lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2, LIDAR_ANGLE_RANGE/2, num = LIDAR_ANGLE_BINS, endpoint=True)
pixels = [["empty"]*300]*300
#### End of Part 1 #####

def world_to_display(x, y, color, pixels):
	if x>1 or x<0 or y>1 or y<0:
		return
	colors = {"blue":0x0000FF,"white":0xFFFFFF,"red":0xFF0000}
	const = 300
	x_disp, y_disp = None, None
	x_disp, y_disp = int(const * x), int(const * y)
	if color=="white" and pixels[x_disp][y_disp]!="blue" and pixels[x_disp][y_disp]!="red": #prevent overwriting
		display.setColor(colors[color])
		display.drawPixel(x_disp, y_disp)
		pixels[x_disp][y_disp] = color
	else:
		display.setColor(colors[color])
		display.drawPixel(x_disp, y_disp)
		pixels[x_disp][y_disp] = color


def ray_to_white(start, stop, pixels):

	if abs(start[0]-stop[0])>abs(start[1] - stop[1]):
		x_range = np.linspace(start[0], stop[0], num=50, endpoint=False)#300*int(abs(start[0]-stop[0])))
		y_range = np.linspace(start[1], stop[1], num=50, endpoint=False)#300*int(abs(start[0]-stop[0])))
	else:
		x_range = np.linspace(start[0], stop[0], num=50, endpoint=False)#300*int(abs(start[1]-stop[1])))
		y_range = np.linspace(start[1], stop[1], num=50, endpoint=False)#300*int(abs(start[1]-stop[1])))
	for i in range(len(x_range)):
		world_to_display(x_range[i], y_range[i], "white", pixels)

	
def lidar_to_world(lidar_sensor_readings, lidar_offsets, x, y, theta, pixels):
	#returns an array of world coordinates that represent obstacles
	for i in range(len(lidar_sensor_readings)):
		if lidar_sensor_readings[i]!=np.inf:
			hyp = lidar_sensor_readings[i]
			x_rob = hyp * np.sin(-lidar_offsets[i])
			y_rob = hyp * np.cos(-lidar_offsets[i])
			rotate_mat = np.array([
									[np.cos(-theta),-np.sin(-theta), 0, x],
									[np.sin(-theta), np.cos(-theta), 0, y],
									[0,0,1,theta],
									[0,0,0,1]])
			QR = np.array([[x_rob],[y_rob],[0],[1]])
			QO = np.matmul(rotate_mat, QR)
			world_to_display(QO[0][0],QO[1][0], "blue", pixels)
			ray_to_white((x,y), (QO[0][0],QO[1][0]), pixels)

 
# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:     
	
	#####################################################
	#                 Sensing                           #
	#####################################################

	# Read ground sensors
	for i, gs in enumerate(ground_sensors):
		gsr[i] = gs.getValue()

	# Read Lidar           
	lidar_sensor_readings = lidar.getRangeImage()
	
	#print("lidar: ", lidar_sensor_readings)
	##### Part 2: Turn world coordinates into map coordinates
	#
	# Come up with a way to turn the robot pose (in world coordinates)
	# into coordinates on the map. Draw a red dot using display.drawPixel()
	# wherehere the robot moves.
	

	
	
	##### Part 3: Convert Lidar data into world coordinates
	#
	# Each Lidar reading has a distance rho and an angle alpha.
	# First compute the corresponding rx and ry of where the lidar
	# hits the object in the robot coordinate system. Then convert
	# rx and ry into world coordinates wx and wy. This lab uses
	# the Webots coordinate system (except that we use Y instead of Z).
	# The arena is 1x1m2 and its origin is in the top left of the arena. 
	

	lidar_to_world(lidar_sensor_readings, lidar_offsets, pose_x, pose_y, pose_theta, pixels)

	
	##### Part 4: Draw the obstacle and free space pixels on the map
 
	
		  
	world_to_display(pose_x, pose_y, "red", pixels)
	
 

	
	# DO NOT MODIFY THE FOLLOWING CODE
	#####################################################
	#                 Robot controller                  #
	#####################################################

	if state == "line_follower":
			if(gsr[1]<350 and gsr[0]>400 and gsr[2] > 400):
				vL=MAX_SPEED*0.3
				vR=MAX_SPEED*0.3                
			# Checking for Start Line          
			elif(gsr[0]<500 and gsr[1]<500 and gsr[2]<500):
				vL=MAX_SPEED*0.3
				vR=MAX_SPEED*0.3
				# print("Over the line!") # Feel free to uncomment this
				display.imageSave(None,"map.png") 
			elif(gsr[2]<650): # turn right
				vL=0.2*MAX_SPEED
				vR=-0.05*MAX_SPEED
			elif(gsr[0]<650): # turn left
				vL=-0.05*MAX_SPEED
				vR=0.2*MAX_SPEED
			 
	else:
		# Stationary State
		vL=0
		vR=0   
	
	leftMotor.setVelocity(vL)
	rightMotor.setVelocity(vR)
	
	#####################################################
	#                    Odometry                       #
	#####################################################
	
	EPUCK_MAX_WHEEL_SPEED = 0.11695*SIM_TIMESTEP/1000.0 
	dsr=vR/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
	dsl=vL/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
	ds=(dsr+dsl)/2.0
	
	pose_y += ds*math.cos(pose_theta)
	pose_x += ds*math.sin(pose_theta)
	pose_theta += (dsr-dsl)/EPUCK_AXLE_DIAMETER
	
	# Feel free to uncomment this for debugging
	print("X: %f Y: %f Theta: %f " % (pose_x,pose_y,pose_theta))
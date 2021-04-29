"""image_recognize_test controller."""

# get the time step of the current world.
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard, CameraRecognitionObject
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink


MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
# part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
# 			"arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
# 			"arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# # All motors except the wheels are controlled by position control. The wheels
# # are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')


part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
			"arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
			"arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")


robot_parts = []

color_dict = {1 : 0.2, 2 : 0.6, 3 : 0.8}

for i in range(N_PARTS):
	robot_parts.append(robot.getDevice(part_names[i]))
	if i > 9:
		robot_parts[i].setPosition(float(target_pos[i]))
		robot_parts[i].setVelocity(0)

# # The Tiago robot has a couple more sensors than the e-Puck
# # Some of them are mentioned below. We will use its LiDAR for Lab 5

# # range = robot.getDevice('range-finder')
# # range.enable(timestep)

camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()
lidar_sensor_readings = []

lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # remove blocked sensor rays


# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# # The display is used to display the map. We are using 360x360 pixels to
# # map the 12x12m2 apartment

display = robot.getDevice("display")

# Odometry
pose_x = 4.47
pose_y = 8.06
pose_theta = -1.61988

vL = 2
vR = 2


conversion_factor = 30

# # You should insert a getDevice-like function in order to get the
# # instance of a device of the robot. Something like:
# #  motor = robot.getMotor('motorname')
# #  ds = robot.getDistanceSensor('dsname')
# #  ds.enable(timestep)

mode = ''#'manual'

map_draw = np.zeros((210, 210))
map_color = np.zeros((210, 210))

code = "20982"

base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_" + code + "_joint", "TIAGo front arm_" + code]

filename = "tiago_urdf.urdf"

my_chain = Chain.from_urdf_file(filename, base_elements=base_elements)

print(my_chain.links)

vector =[0.003999353, 0, -0.1741]

my_chain = Chain.from_urdf_file(filename, last_link_vector=vector, base_elements=base_elements)

print(my_chain.links)

hands = ("gripper_left_finger_joint", "gripper_right_finger_joint")

hand_parts = []
for i in range(2):
	hand_parts.append(robot.getDevice(hands[i]))
	hand_parts[i].setPosition(float(0.05))

for link_id in range(len(my_chain.links)):

	# This is the actual link object
	link = my_chain.links[link_id]
	
	# I've disabled "torso_lift_joint" manually as it can cause
	# the TIAGO to become unstable.
	if link.name not in part_names or link.name =="torso_lift_joint":
		print("Disabling {}".format(link.name))
		my_chain.active_links_mask[link_id] = False


motors = []
for link in my_chain.links:
	if link.name in part_names and link.name != "torso_lift_joint":
		motor = robot.getDevice(link.name)

		# Make sure to account for any motors that
		# require a different maximum velocity!
		if link.name == "torso_lift_joint":
			motor.setVelocity(0.07)
		else:
			motor.setVelocity(1)
			
		position_sensor = motor.getPositionSensor()
		position_sensor.enable(timestep)
		motors.append(motor)

print("done disabling")

initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]

print("initial_positions")


def draw_map(map_draw, map_color):
	map_draw_c = (map_draw >0.5)*1
	map2 = np.zeros((210,210))
	for i in range(len(map_draw_c)):
		for j in range(len(map_draw_c[i])):
			if map_draw_c[i][j] == 1:
				color = map_color[i][j]
				for x in range(max(0, i-3), min(210, i+3)):
					for y in range(max(0, j-3), min(210, j+3)):
						map2[x][y] = 1 if color == 0 else color_dict[color]
	print("Map Configured")
	plt.imshow(np.fliplr(np.rot90(map_color,k=3)))
	plt.show()

	plt.imshow(np.fliplr(np.rot90(map2,k=3)))
	plt.show()

def rrt(state_bounds, state_is_valid, starting_point, goal_point, k, delta_q):
	'''
	TODO: Implement the RRT algorithm here, making use of the provided state_is_valid function.
	RRT algorithm.
	If goal_point is set, your implementation should return once a path to the goal has been found
	(e.g., if q_new.point is within 1e-5 distance of goal_point), using k as an upper-bound for iterations.
	If goal_point is None, it should build a graph without a goal and terminate after k iterations.

	:param state_bounds: matrix of min/max values for each dimension (e.g., [[0,1],[0,1]] for a 2D 1m by 1m square)
	:param state_is_valid: function that maps states (N-dimensional Real vectors) to a Boolean (indicating free vs. forbidden space)
	:param starting_point: Point within state_bounds to grow the RRT from
	:param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
	:param k: Number of points to sample
	:param delta_q: Maximum distance allowed between vertices
	:returns List of RRT graph nodes
	'''
	node_list = []
	node_list.append(Node(starting_point, parent=None))  # Add Node at starting point with no parent

	# TODO: Your code here
	# TODO: Make sure to add every node you create onto node_list, and to set node.parent and node.path_from_parent for each

	for i in range(0, k):
		randPoint = get_random_valid_vertex(state_is_valid, state_bounds)
		if goal_point is not None:
			chanceTowardsGoal = random.random()
			if chanceTowardsGoal < 0.1:
				randPoint = goal_point
		closestNode = get_nearest_vertex(node_list, randPoint)
		currentPath = steer(closestNode.point, randPoint, delta_q)
		if (check_path_valid(currentPath, state_is_valid) == True):
			newNode = Node(currentPath[len(currentPath)-1], closestNode)
			newNode.path_from_parent = currentPath
			node_list.append(newNode)

	return node_list

i = 0

pos = (1.6, 0.4, 0, 0, 0.1, 0.15, 1.5)
pos2 = (1.6, 0.55, 0, 1.4, 0.1, 0.15, 1.5)

states= {'picked_up':False}

path = [(6.13,4),(6.3,4.77),(5.53,5.14),(5.76,5.3)]
state = 0
p1 = 2
p2 = -1
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
	pose_y = gps.getValues()[2]
	pose_x = gps.getValues()[0]

	# print(pose_x, pose_y)

	n = compass.getValues()
	rad = -((math.atan2(n[0], n[2])) + 3.14) #-1.5708
	pose_theta = rad
	if not states['picked_up']:
		i+=1

		if i < 20:
			for part in range(4, len(part_names)-1):
				robot.getDevice(my_chain.links[part].name).setPosition(pos[part-4])
				print("Setting {} to {}".format(my_chain.links[part].name, pos[part-4]))
		if i > 20 and i < 50:
			for part in range(4, len(part_names)-1):
				robot.getDevice(my_chain.links[part].name).setPosition(pos2[part-4])
				print("Setting {} to {}".format(my_chain.links[part].name, pos2[part-4]))

		if i > 60 and i < 80:
			for part in range(2):
				hand_parts[part].setPosition(float(0.01))

		if i>80:
			offset_target =[-2,-2,-5]
			if i == 81:
				ikResults = my_chain.inverse_kinematics(offset_target, initial_position=[0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0],  target_orientation = [0,0,1], orientation_mode="Y")

			if i>82:
				error = 0
				for item in range(3):
					error += (offset_target[item] - ikTarget[item])**2 #ikTarget is the previous target used for calculations.
				error = math.sqrt(error)
				if error > 0.5:
					ikResults = my_chain.inverse_kinematics(offset_target, initial_position=[0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0],  target_orientation = [0,0,1], orientation_mode="Y")
			ikTarget = offset_target

			print("error calculations")
			for res in range(len(ikResults)):
				# This if check will ignore anything that isn't controllable
				if my_chain.links[res].name in part_names:
					robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
					print("Setting {} to {}".format(my_chain.links[res].name, ikResults[res]))
			states['picked_up'] = True
	else:



	# quit()
# 	# Read the sensors:
# 	# Enter here functions to read sensor data, like:
# 	#  val = ds.getValue()

# 	# Process sensor data here.

# 	# Enter here functions to send actuator commands, like:
# 	# motor.setPosition(10.0)
		lidar_sensor_readings = lidar.getRangeImage()
		lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

		# print(pose_x, pose_y, pose_theta)

		for i, rho in enumerate(lidar_sensor_readings):
			alpha = lidar_offsets[i]

			if rho > LIDAR_SENSOR_MAX_RANGE:
				rho = LIDAR_SENSOR_MAX_RANGE

			rx = math.cos(alpha)*rho
			ry = -math.sin(alpha)*rho

			wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
			wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y

			#print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

			if rho < 0.3*LIDAR_SENSOR_MAX_RANGE:
	# Part 1.3: visualize map gray values.
				try:
					map_draw[210-int(wy*conversion_factor),int(wx*conversion_factor)] += 0.01
					g = map_draw[210-int(wy*conversion_factor),int(wx*conversion_factor)]
					g = 1 if g>1 else g
					c = int(((g*(256**2))+(g*256)+g)*255)
					# You will eventually REPLACE the following 2 lines with a more robust version of map
					# and gray drawing that has more levels than just 0 and 1.
					display.setColor(c)
					display.drawPixel(210-int(wy*conversion_factor),int(wx*conversion_factor))
				except:
					pass

			if i%3 == 0 and abs(round(alpha, 2)) < 0.2:
				obj = camera.getRecognitionObjects()
				try:
					obj = obj[0]
					if obj.get_colors()[0] == 1:
						map_color[210-int(wy*conversion_factor)][int(wx*conversion_factor)] = 1
					elif obj.get_colors()[1] == 1:
						map_color[210-int(wy*conversion_factor)][int(wx*conversion_factor)] = 2
					elif obj.get_colors()[2] == 1:
						map_color[210-int(wy*conversion_factor)][int(wx*conversion_factor)] = 3
					
				except:
					pass
		i+=1
		display.setColor(int(0xFF0000))
		display.drawPixel(210-int(pose_y*conversion_factor),int(pose_x*conversion_factor))

		if mode == 'manual':
			key = keyboard.getKey()
			while(keyboard.getKey() != -1): pass
			if key == keyboard.LEFT :
				vL = -MAX_SPEED
				vR = MAX_SPEED
			elif key == keyboard.RIGHT:
				vL = MAX_SPEED
				vR = -MAX_SPEED
			elif key == keyboard.UP:
				vL = MAX_SPEED
				vR = MAX_SPEED
			elif key == keyboard.DOWN:
				vL = -MAX_SPEED
				vR = -MAX_SPEED
			elif key == ord(' '):
				vL = 0
				vR = 0
			elif key == ord('S'):
				draw_map(map_draw, map_color)
			else: # slow down
				vL *= 0.75
				vR *= 0.75
			pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
			pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
			pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

			robot_parts[MOTOR_LEFT].setVelocity(vL)
			robot_parts[MOTOR_RIGHT].setVelocity(vR)
		else:
			xg, yg = path[state]
			#goal points and error
			rho = math.sqrt((xg-pose_x)**2 + (yg-pose_y)**2)
			alpha = math.atan2(yg-pose_y, xg-pose_x) - pose_theta

			#STEP 2: Controller

			x_r = p1 * rho
			theta_r = p2 * alpha

			#STEP 3: Compute wheelspeeds

			vL = (-(AXLE_LENGTH * theta_r)+(2*x_r))/2
			vR = ((AXLE_LENGTH * theta_r)+(2*x_r))/2

		# Normalize wheelspeed
		# Keep the max speed a bit less to minimize the jerk in motion

			if abs(vL) > MAX_SPEED:
				vL = MAX_SPEED
			if abs(vR) > MAX_SPEED:
				vR = MAX_SPEED

			if rho < 0.5:
				state +=1
				print("state change to {}".format(state))
				vL = vR = 0
				continue

			# Odometry code. Don't change speeds after this
			# We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
			pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
			pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
			pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

			print("X: %f Z: %f Theta: %f, state: %f, rho: %f" % (pose_x, pose_y, pose_theta, state, rho)) #/3.1415*180))

		# Actuator commands
		robot_parts[MOTOR_LEFT].setVelocity(vL)
		robot_parts[MOTOR_RIGHT].setVelocity(vR)


	# print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta)) #/3.1415*180))

	# try:
	# 	x = camera.getRecognitionObjects()
	# 	for ind, obj in enumerate(x):
	# 		print(ind, " " ,obj.get_colors())
	# 	# print("Object position: ", pose_x + x[0], ", ", pose_y + x[2] )
	# except:
	# 	pass

	pass
	# Actuator commands
	

# Enter here exit cleanup code.

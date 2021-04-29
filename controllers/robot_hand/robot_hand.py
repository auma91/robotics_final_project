"""ikpy controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# with open("tiago_urdf.urdf", "w") as file:
#	file.write(robot.getUrdf())

code = "20982"

base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_" + code + "_joint", "TIAGo front arm_" + code]

filename = "tiago_urdf.urdf"

my_chain = Chain.from_urdf_file(filename, base_elements=base_elements)

print(my_chain.links)

vector =[0.003999353, 0, -0.1741]

my_chain = Chain.from_urdf_file(filename, last_link_vector=vector, base_elements=base_elements)

print(my_chain.links)


part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
			"arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
			"arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

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

recognized_objects = camera.getRecognitionObjects() # Refer to the recognition section is Webots doc for more info on this
target = recognized_objects[0].get_position() # This is the position of the target in camera coordinates
offset_target = [-(target[2])+0.22, -target[0]+0.08, (target[1])+0.97+0.2] # And here it is translated to robot/IK coordinates
# Note that the `+0.2` is optional, and since Z is up, this raises the target IK by 20cm for my solution.


ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")

error = 0
for item in range(3):
	error += (offset_target[item] - ikTarget[item])**2 #ikTarget is the previous target used for calculations.
error = math.sqrt(error)

print("error calculations")
for res in range(len(ikResults)):
	# This if check will ignore anything that isn't controllable
	if my_chain.links[res].name in part_names:
		robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
		print("Setting {} to {}".format(my_chain.links[res].name, ikResults[res]))

print("done")
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
	# Read the sensors:
	# Enter here functions to read sensor data, like:
	#  val = ds.getValue()

	# Process sensor data here.

	# Enter here functions to send actuator commands, like:
	#  motor.setPosition(10.0)
	pass

# Enter here exit cleanup code.

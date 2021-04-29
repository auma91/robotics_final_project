	# recognized_objects = camera.getRecognitionObjects() # Refer to the recognition section is Webots doc for more info on this
	# target = recognized_objects[0].get_position() # This is the position of the target in camera coordinates
	# offset_target = [-(target[2])+0.22, -target[0]+0.08, (target[1])+0.97+0.2] # And here it is translated to robot/IK coordinates
	# # Note that the `+0.2` is optiona1l, and since Z is up, this raises the target IK by 20cm for my solution.


	# if i == 0:
	# 	ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")

	# if i>0:
	# 	error = 0
	# 	for item in range(3):
	# 		error += (offset_target[item] - ikTarget[item])**2 #ikTarget is the previous target used for calculations.
	# 	error = math.sqrt(error)
	# 	if error > 0.5:
	# 		ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position,  target_orientation = [0,0,1], orientation_mode="Y")
	# ikTarget = offset_target

	# print("error calculations")
	# for res in range(len(ikResults)):
	# 	# This if check will ignore anything that isn't controllable
	# 	if my_chain.links[res].name in part_names:
	# 		robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
	# 		print("Setting {} to {}".format(my_chain.links[res].name, ikResults[res]))
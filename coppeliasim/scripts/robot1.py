#python

import math

robot = None
obstacles = None
usensors = []
motorLeft = None
motorRight = None
noDetectionDist = 0.5
maxDetectionDist = 0.2
detect = [0] * 16
braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6] + [0] * 8
braitenbergR = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2] + [0] * 8
v0 = 2
# Add a weight factor for the Braitenberg algorithm (adjust as needed)
braitenberg_weight = 1

def sysCall_init():
    global robot, usensors, motorLeft, motorRight, obstacles, target

    robot = sim.getObject('.')
    obstacles = sim.createCollection(0)
    sim.addItemToCollection(obstacles, sim.handle_all, -1, 0)
    sim.addItemToCollection(obstacles, sim.handle_tree, robot, 1)

    # Create the target sphere
    target = sim.createPrimitiveShape(sim.primitiveshape_spheroid, [0.05,0.05,0.05], 0)
    sim.setObjectAlias(target, "target1")
    sim.addLog(0, f'target: {target} alias: {sim.getObjectAlias(target)}')

    # Set the target sphere color to red
    sim.setShapeColor(target, None, sim.colorcomponent_ambient_diffuse, [1, 0, 0])

    # Set the target sphere as a child of the parent robot
    #sim.setObjectParent(target, robot, True)

    motorLeft = sim.getObject("./leftMotor")
    motorRight = sim.getObject("./rightMotor")

    for i in range(16):
        sensor = sim.getObject("./ultrasonicSensor", {"index": i})
        sim.setObjectInt32Param(sensor, sim.proxintparam_entity_to_detect, obstacles)
        usensors.append(sensor)

    motorLeft = sim.getObject("./leftMotor")
    motorRight = sim.getObject("./rightMotor")
    sim.addLog(0, f'robot: {robot}')


def sysCall_cleanup():
    pass


def sysCall_actuation():
    global detect, v0

    # Avoid other robots
    detected_robots = avoid_robots()

    # Get the sphere object handle
    #target = sim.getObject("./target1")
    target = sim.getObject("/target1")

    # Get the sphere position
    target_pos = sim.getObjectPosition(target, -1)

    # Calculate the robot's position and orientation
    robot_pos = sim.getObjectPosition(robot, -1)
    robot_orient = sim.getObjectOrientation(robot, -1)
    
    # Calculate the angle between the robot's position and the sphere position
    angle_to_target = math.atan2(target_pos[1] - robot_pos[1], target_pos[0] - robot_pos[0])

    # Calculate the angle error between the robot's orientation and the angle to the sphere position
    angle_error = angle_to_target - robot_orient[2]

    # Normalize the angle error to the range [-pi, pi]
    while angle_error > math.pi:
        angle_error -= 2 * math.pi
    while angle_error < -math.pi:
        angle_error += 2 * math.pi

    # Calculate the distance to the target
    distance_to_target = math.sqrt((target_pos[0] - robot_pos[0])**2 + (target_pos[1] - robot_pos[1])**2)

    # Scale the base speed based on the distance to the target
    scaled_v0 = v0 * (1 + distance_to_target)

    # Calculate the base speed for each wheel
    vLeft = scaled_v0 - angle_error * 2
    vRight = scaled_v0 + angle_error * 2

    if detected_robots:
        # If there are detected robots, adjust the wheel speeds accordingly
        front_robot_detected = any(i < 8 for i, _ in detected_robots)
        side_robot_detected = any(i >= 8 for i, _ in detected_robots)
        if front_robot_detected:
            vLeft += v0
            vRight -= v0
        elif side_robot_detected:
            vLeft += 1.5 * v0  # Increase the speed difference between the wheels for a sharper turn
            vRight -= 1.5 * v0  # Increase the speed difference between the wheels for a sharper turn
        else:
            for i, dist in detected_robots:
                vLeft += braitenbergL[i] * detect[i] * 2
                vRight += braitenbergR[i] * detect[i] * 2
    else:
        # Implement the Braitenberg algorithm
        braitenberg_left = 0
        braitenberg_right = 0
        for i in range(16):
            braitenberg_left += braitenbergL[i] * detect[i]
            braitenberg_right += braitenbergR[i] * detect[i]

        # Combine the path following and obstacle avoidance behaviors using the weight factor
        vLeft += braitenberg_weight * braitenberg_left
        vRight += braitenberg_weight * braitenberg_right

    # Set wheel velocities
    sim.setJointTargetVelocity(motorLeft, vLeft)
    sim.setJointTargetVelocity(motorRight, vRight)


def avoid_robots():
    detected_robots = []

    for i in range(16):
        res, dist, point, obj_handle, norm = sim.readProximitySensor(usensors[i])
        front_detection_dist = noDetectionDist * 1.5 if i < 8 else noDetectionDist
        if res > 0 and dist < front_detection_dist:
            if dist < maxDetectionDist:
                dist = maxDetectionDist
            detect[i] = 1 - ((dist - maxDetectionDist) / (front_detection_dist - maxDetectionDist))

            # Check if the detected obstacle is another robot
            obj_type = sim.getObjectType(obj_handle)
            if obj_type == sim.object_shape_type:  # Check if the detected object is a shape
                obj_name = sim.getObjectAlias(obj_handle)
                if "Robot" in obj_name:  # Check if the detected shape has "Robot" in its name
                    detected_robots.append((i, dist))

    return detected_robots

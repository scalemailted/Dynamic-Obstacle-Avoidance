#python

import math
import numpy as np

def sysCall_thread():
    sim.setThreadAutomaticSwitch(True)
    print('threaded script')
    sim.addLog(0, "threaded script")
    dt = sim.getSimulationTimeStep() * 0.15 
    print(f'dt: {dt}')

    # Wait for the target sphere objects and robot objects to be created
    target1 = None
    target2 = None
    robot1 = None
    robot2 = None
    while target1 is None or target2 is None or robot1 is None or robot2 is None:
        target1 = sim.getObject('/target1')
        target2 = sim.getObject('/target2')
        robot1 = sim.getObject('/Robot1')
        robot2 = sim.getObject('/Robot2')
        sim.addLog(0, f'target1:{target1} target2:{target2} Robot1:{robot1} Robot2:{robot2}')
        sim.wait(dt)

    catch_up_distance = 1

    while True:
        pathHandle = sim.getInt32Signal('pathHandle')
        if pathHandle:
            sim.clearInt32Signal('pathHandle')
            path_data = sim.readCustomDataBlock(pathHandle, 'PATH')
            path = sim.unpackDoubleTable(path_data)
            pathMat = np.array(path).reshape(len(path) // 7, 7)
            pathMat_reversed = np.flip(pathMat, axis=0)  # Reversed path for Robot2
            count1 = 0
            count2 = 0
            #
            while count1 < len(pathMat) - 1 or count2 < len(pathMat_reversed) - 1:
                if count1 < len(pathMat) - 1:
                    current_pose = pathMat[count1]
                    robot1_pos = sim.getObjectPosition(robot1, -1)
                    target1_pos = sim.getObjectPosition(target1, -1)
                    distance_to_target = np.linalg.norm(np.array(target1_pos) - np.array(robot1_pos))
                    if distance_to_target < catch_up_distance:
                        count1 += 1
                    next_pose = pathMat[count1]
                    # Calculate the desired orientation for the robot based on the direction to the next pose
                    desired_orientation = calculate_orientation(current_pose[:3], next_pose[:3])
                    # Move the sphere to the current pose
                    sim.setObjectPosition(target1, -1, current_pose[:3].tolist())
                    
                if count2 < len(pathMat_reversed) - 1:
                    current_pose = pathMat_reversed[count2]
                    robot2_pos = sim.getObjectPosition(robot2, -1)
                    target2_pos = sim.getObjectPosition(target2, -1)
                    distance_to_target = np.linalg.norm(np.array(target2_pos) - np.array(robot2_pos))
                    
                    if distance_to_target < catch_up_distance:
                        count2 += 1
                    # For Robot2, use pathMat_reversed
                    next_pose = pathMat_reversed[count2]
                    # Calculate the desired orientation for Robot2 based on the direction to the next pose
                    desired_orientation = calculate_orientation(current_pose[:3], next_pose[:3])
                    # Move the sphere for Robot2 to the current pose
                    sim.setObjectPosition(target2, -1, current_pose[:3].tolist())       
                    
                sim.wait(dt)
            

def calculate_orientation(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    yaw = math.atan2(dy, dx)
    # Assuming the robot moves on a 2D plane (X, Y) with a fixed Z-axis orientation
    return [0, 0, yaw]

#python

###########################################################

#dstarAlgoHW5

import numpy as np
from sklearn.preprocessing import MinMaxScaler
import sys
import math

altitude = 0.1


def populate_objects(grid_res, obstacles):
    '''
    parameter
    ---------
    @obstacles : np.ndarray (n x 2) array
    a list of coordinates for 2D obstacles
    '''
    primitiveType = sim.primitiveshape_cuboid
    OBSTACLE_HEIGHT = 1 #m
    sizes = [grid_res, grid_res, OBSTACLE_HEIGHT] #m
    default_z = grid_res #m

    shapeHandles = []
    for obs in obstacles:
        # create a cube object
        shapeHandle = sim.createPrimitiveShape(primitiveType, sizes)
        # change its position from (0,0,0) to the obstacles position
        sim.setObjectPosition(shapeHandle, -1, [obs[0], obs[1], default_z])
        # append this shapeHandle to the shapeHandles list so that we can merge it
        shapeHandles.append(shapeHandle)
    groupHandle = sim.groupShapes(shapeHandles, True)
    #default name is cuboid. Change it to Obstacles.
    sim.setObjectAlias(groupHandle,"Obstacles")
    #sim.ObjectAlias(groupHandle, "Obstacles")
    print(f"groupHandle: {groupHandle}")


def getObjectPosition(objName):
    '''
        parameter
        =========
        objName : str
            name of start or goal objects
        return
            2D position
    '''
    handle = sim.getObject(objName)
    return sim.getObjectPosition(handle, -1)[:2]

def getPath(manifold, robot, dest):
    '''
        parameter
        ==========
        manifold: CoppeliaSimManifold
            mapping between Discrete Grid and Continuous World
        robot: list
            start position of robot
        dest: list
            position of goal
        return
            a set of waypoints in Continuous World

    '''

    GRID_RESOLUTION = 100
    DILATION_VALUE = 4 #sim.getInt32Signal("safety")
    print(f"dilation = {DILATION_VALUE}")
    m = DilatedMap(GRID_RESOLUTION, GRID_RESOLUTION, DILATION_VALUE)

    m.set_obstacle(manifold.obstacles)

    # convert to planning domain coordinate
    start = manifold.getPlannerCoord(robot)
    goal = manifold.getPlannerCoord(dest)
    print(f"start = {start} goal = {goal}")
    start = m.map[ start[0]][start[1]]
    end = m.map[goal[0]][goal[1]]

    #compute shortest path using DStar algorithm
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)
    #append goal location
    rx.append(goal[0])
    ry.append(goal[1])
    # convert path to coppelia coordinate
    path = np.column_stack((rx, ry))
    pathCoppelia = manifold.transformPath(path)
    pathCoppelia[0, :] = robot

    return pathCoppelia

def drawPath(path):
    lineSize = 5 #-- in points
    maximumLines = 9999
    red = [1,0,0]


    drawingObjectHandle = sim.addDrawingObject(sim.drawing_lines, lineSize, 0.0, -1, maximumLines)

    for i, p in enumerate(path):
        if i > 0:
            last = path[i-1].tolist()
            last.append(altitude)
            # convert numpy array to list
            curr = p.tolist()
            curr.append(altitude)
            #line requires 6 elements, i.e., two 3D points
            last.extend(curr)
            # draw line on the floor
            sim.addDrawingObjectItem(drawingObjectHandle, last)


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quarternion.

    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def createTraj(path):

    ctrlPts = []
    vxmin = vymin = vzmin = 0.0
    vxmax = vymax = vzmax = 2.0
    ppath = []
    minMaxVel = minMaxAccel = []
    for p in path:
        yaw = math.atan2(p[1], p[0])
        pose = [p[0], p[1], altitude]
        ori = get_quaternion_from_euler(0, 0, yaw)
        ppath.extend(pose.copy())
        pose.extend(ori)
        ctrlPts.extend(pose)

    v = [vxmin, vxmax, vymin, vymax, vzmin, vzmax]
    minMaxVel.extend(v)

    #print(ctrlPts)
    #pathHandle

    pathLengths, totalLength = sim.getPathLengths(ctrlPts, 7)
    dt = sim.getSimulationTimeStep()
    print(totalLength)
    pathHandle = sim.createPath(ctrlPts, 0, totalLength // dt)
    sim.setInt32Signal('pathHandle', pathHandle)

    #outputPath



def sysCall_init():
    set_floor_size()
    obs = get_obstacles()
    #min and max index of the floor
    feature_range = (-5, 5)
    manifold = CoppeliaSimManifold(feature_range)

    res = manifold.transform(obs)
    populate_objects(0.25,res)

    #robot = getObjectPosition("/PioneerP3DX")
    robot = getObjectPosition("/Goal2")
    dest = getObjectPosition("/Goal1")
    path = getPath(manifold, robot, dest)
    createTraj(path)



def set_floor_size():
    # Get the floor object handle
    floor = sim.getObject("/Floor/box")
    if floor != -1:
        # Set the floor size
        floor_length = 10
        floor_width = 10
        floor_height = 0.01
        set_object_size(floor, floor_width, floor_width, floor_height)
        # Set the floor position (assuming the Z position is 0)
        sim.setObjectPosition(floor, -1, [0, 0, -0.3])
        
    else:
        sim.addLog(0, 'Floor object not found')

def set_object_size(handle, x, y, z):
    r, min_x = sim.getObjectFloatParameter(handle, sim.objfloatparam_objbbox_min_x)
    r, max_x = sim.getObjectFloatParameter(handle, sim.objfloatparam_objbbox_max_x)
    sx = max_x - min_x
    
    r, min_y = sim.getObjectFloatParameter(handle, sim.objfloatparam_objbbox_min_y)
    r, max_y = sim.getObjectFloatParameter(handle, sim.objfloatparam_objbbox_max_y)
    sy = max_y - min_y

    r, min_z = sim.getObjectFloatParameter(handle, sim.objfloatparam_objbbox_min_z)
    r, max_z = sim.getObjectFloatParameter(handle, sim.objfloatparam_objbbox_max_z)
    sz = max_z - min_z
    
    #sim.scaleObject(handle, x / sx, y / sy, z / sz)
    sim.scaleObject(handle, x / sx, y / sy, 1)


    
###########################################################

#from dstarAlgo import Dstar

"""

D* grid planning

author: Nirnay Roy

See Wikipedia article (https://en.wikipedia.org/wiki/D*)

"""
import math

from sys import maxsize


class State:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"  # tag for state
        self.h = 0
        self.k = 0

    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return maxsize

        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    def set_state(self, state):
        """
        .: new
        #: obstacle
        e: oparent of current state
        *: closed state
        s: current state
        """
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state


class Map:

    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def get_neighbors(self, state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")


class Dstar:
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()

    def process_state(self):
        x = self.min_state()

        if x is None:
            return -1

        k_old = self.get_kmin()
        self.remove(x)

        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        elif k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(y, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        rx = []
        ry = []

        self.insert(end, 0.0)

        while True:
            self.process_state()
            if start.t == "close":
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start

        while tmp != end:
            tmp.set_state("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            if tmp.parent.state == "#":
                self.modify(tmp)
                continue
            tmp = tmp.parent
        tmp.set_state("e")

        return rx, ry

    def modify(self, state):
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break


###########################################################

#from dstarCoppelia import get_obstacles, CoppeliaSimManifold, DilatedMap


class DilatedMap(Map):
    def __init__(self, row, col, neighbour):
        super().__init__(row, col)
        self.dilated_neighbour = neighbour
    
    def set_obstacle(self, point_list):
        def isInValid(x, y):
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                return True
            else:
                return False
        
        for x, y in point_list:
            if isInValid(x, y):
                continue
            self.map[x][y].set_state("#")
            # dilation: also avoid going to neighboring cells for each obactacle
            neighbour = self.dilated_neighbour
            for i in range(-neighbour, neighbour):
                for j in range(-neighbour, neighbour):
                    nx, ny = x + i, y + j
                    if isInValid(nx, ny):
                        continue
                    self.map[nx][ny].set_state('#')


class CoppeliaSimManifold:
    def __init__(self, feature_range):
        self.scaler = MinMaxScaler(feature_range)
        self.grid_res = 0.2
    

    def transform(self, X):
        self.obstacles = X.copy()
        self.scaler.fit(X)
        return self.scaler.transform(X)
    
    def getCoppeliaCoord(self, x):
        x = np.array(x)
        x = np.expand_dims(x, axis=0)
        return np.squeeze(self.scaler.transform(x)).tolist()
    
    def getPlannerCoord(self, coord):
        x = np.array([coord[0], coord[1]])
        x = np.expand_dims(x, axis=0)
        y = np.squeeze(self.scaler.inverse_transform(x))
        return y.astype('int').tolist()

    def transformPath(self, path):
        return np.squeeze(self.scaler.transform(path))



def get_obstacles():
    '''
    this code is borrowed from
    https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DStar/dstar.py
    '''
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10)
    for i in range(-10, 60):
        ox.append(60)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60)
    for i in range(-10, 61):
        ox.append(-10)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40)
        oy.append(60 - i)

    #return (ox, oy)
    return [(i, j) for i, j in zip(ox, oy)]

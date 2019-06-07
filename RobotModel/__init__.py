from pyrep import VRep
from pyrep.vrep import vrep as v
import RobotControl
import numpy as np
import threading
import functions as f
import time

class RobotState:
    def __init__(self):
        self.block_env = {}
        self.Moving: bool = False
        self.isHolding: int = 0
        self.lastHeld: int = 1
        self.nextEFPosition = (0, 0, 0)
        self.overHome = True
        self.goingToSharedSpace = False
        self.table_env = []  # Only contains free slots
        self.shared_env = []  # Only contains free slots


class RobotModel:
    def __init__(self, name: str, api: VRep, controller: RobotControl):
        self._api = api
        self._name = name
        self._controller = controller
        self._sensors = None
        self._actuators = None
        self._state = None

    def process_commands(self, cmd):
        pass


class Uarm(RobotModel):
    def __init__(self, name, api, controller ):
        RobotModel.__init__(self, name, api,controller)
        self._actuators = {}
        self._sensors = {}
        self._baseSize = 106.5
        self._arm1 = 149.7
        self._arm2 = 160.1
        self._endEffectorDisplacement = (0, -25, 0)
        self._thetaDisplacementRad = (0, -19.3*np.pi/180, -4*np.pi/180)
        self._state = RobotState()
        self._actuators['base_motor'] = api.joint.with_position_control(name+"_motor1")
        self._actuators['first_motor'] = api.joint.with_position_control(name+"_motor2")
        self._actuators['second_motor'] = api.joint.with_position_control(name+"_motor3")
        self._actuators['endEffector_motor'] = api.joint.with_position_control(name+"_motor4")
        self._suctionHandler = name+"_suctionCup"
        self._sensors['cameraTop'] = api.sensor.vision(name+"_visionSensorTop")
        self._sensors['cameraFront'] = api.sensor.vision(name+"_visionSensorFront")
        #self._homePosition = (-530 / 2, 0, 140) if self._name == "uarmL" else (530 / 2, 0, 140)
        self._homePosition = (0,100,140)
        self._cameraResX = 128
        self._cameraResY = 128
        self._table_slots = []
        self._shared_slots = []

        if self._name == "uarmR":
            self._table_slots = [
                (-37, 200),
                (37, 200),
                (111, 200),
                (186, 200)
            ]
            self._shared_slots = [
                #(74, 200),
                (-111, 200),
                (-215, 200),
            ]
        else:
            self._table_slots = [
                (37, 200),
                (-37, 200),
                (-111, 200),
                (-186, 200),
            ]
            self._shared_slots = [
                #.(75, 200),
                (111, 200),
                (221, 200)
            ]

    def set_state_env(self, object_list):
        self._state.block_env = object_list

    def set_holding(self, is_holding):
        self._state.isHolding = is_holding

    def is_tracking(self):
        return self._state.Moving

    def set_tracking(self,tracking):
        self._state.Moving = tracking

    def set_over_home(self,isOverHome):
        self._state.overHome = isOverHome

    def set_last_held(self,last_held):
        self._state.lastHeld = last_held

    def is_holding(self):
        return self._state.isHolding

    def get_sizes(self):
        return self._baseSize, self._arm1, self._arm2, self._endEffectorDisplacement, self._thetaDisplacementRad

    # region Teleor
    def go_home(self):
        self.place_end(self._homePosition)
        self._state.overHome = True

    def pickup(self, block,table):
        self._controller.process_percepts()
        name = 'arm1' if self._name == 'uarmR' else 'arm2'
        print("Block ENV: {}".format(self._state.block_env))
        sv = f.getSimplifiedVision(self._state.block_env, name)
        print("SIMPLIFIED VISION {}: {}".format(self._name,sv))
        msg = []
        msg.append('r_(tracking({}))'.format(name))
        if table == 'shared':
            msg.append('f_(over_home({}))'.format(name))
        self._controller.send_percept(msg)
        msg = []
        b_color = f.index_to_color(block)
        x, y, z = self._state.block_env[b_color]
        scheme = [0,0, 61, 115]
        self.place_end((int(x), int(y), 140))
        self._controller.send_percept(['f_(tracking({}))'.format(name)])
        self.place_end((int(x), int(y), scheme[z]))
        self.enable_suction()
        msg.append('r_(holding({},{}))'.format(name,block))

        if sv[block] == 0:
            msg.append('f_(on_table({},{}))'.format(block,table))
            self._table_slots.append((int(x), int(y)))
        else:
            msg.append('f_(on({},{}))'.format(block, sv[block]))
        self.set_holding(block)
        self._controller.send_percept(msg)
        self.place_end((int(x), int(y), 140))
        self._controller.send_percept(['r_(tracking({}))'.format(name)])
        self.go_home()
        self._controller.send_percept(['f_(tracking({}))'.format(name)])
        #self._controller.process_percepts()



    def put_on_block(self, block, table):
        name = 'arm1' if self._name == 'uarmR' else 'arm2'
        holding = self.is_holding()
        self._controller.send_percept(['r_(tracking({}))'.format(name)])
        b_color = f.index_to_color(block)
        if table == 'shared':
            x, y, z = self._state.shared_env[b_color]
            self.set_over_home(False)
            self._controller.send_percept(['f_(over_home({}))'.format(name)])
        else:
            self.set_over_home(True)
            self._controller.send_percept(['r_(over_home({}))'.format(name)])
            x, y, z = self._state.block_env[b_color]
        scheme = [0, 61, 115]
        self.place_end((int(x), int(y), 180))
        self._controller.send_percept(['f_(tracking({}))'.format(name)])
        self.place_end((int(x), int(y), scheme[z]))
        self.disableSuction()
        self._controller.send_percept(['f_(holding({},{}))'.format(name,holding),'r_(on({},{}))'.format(holding,block)])
        self.place_end((int(x), int(y), 180))
        self._controller.send_percept(['r_(tracking({}))'.format(name)])
        self.go_home()
        self.set_tracking(False)
        self.set_last_held(self.is_holding())
        self.set_holding(0)
        self._controller.process_percepts()

    def put_on_table(self, table):
        name = 'arm1' if self._name == 'uarmR' else 'arm2'
        holding = self.is_holding()
        self._controller.send_percept(['r_(tracking({}))'.format(name)])
        if table == 'shared':
            self.set_over_home(False)
            self._controller.send_percept(['f_(over_home({}))'.format(name)])
            x, y = self._state.shared_env.pop(0)  # Use the first free slot and removes it, as it will no longer be free
        else:
            self.set_over_home(True)
            self._controller.send_percept(['r_(over_home({}))'.format(name)])
            x, y = self._state.table_env.pop(0)  # Use the first free slot and removes it, as it will no longer be free
        self.place_end((x, y, 140))
        self._controller.send_percept(['f_(tracking({}))'.format(name)])
        self.place_end((x, y, 5))
        self.disableSuction()
        self._controller.send_percept(['f_(holding({},{}))'.format(name, holding), 'r_(on_table({},{}))'.format(holding, table)])
        self.place_end((x, y, 140))
        #self._controller.send_percept(['r_(tracking({}))'.format(name)])
        self.go_home()
        #self._controller.send_percept(['r_(over_home({}))'.format(name)])
        #self._controller.send_percept(['f_(tracking({}))'.format(name)])
        self.set_last_held(self.is_holding())
        self.set_holding(0)
        self.set_over_home(True)
        self._controller.process_percepts()

    # endregion Teleor

    def place_end(self, coordinates: tuple):
        x, y, z = coordinates
        theta = self.get_motors_theta(x, y, z)
        self._state.Moving = True
        self.rotate_motors(*theta)

    def rotate_motors(self, theta1, theta2, theta3, orient = False):
        self._state.Moving = True

        t1: float = int(theta1 * 100) / 100
        t2: float = int(theta2 * 100) / 100
        t3: float = int(theta3 * 100) / 100

        self._actuators['base_motor'].set_target_position(theta1)
        self._actuators['first_motor'].set_target_position(theta2)
        self._actuators['second_motor'].set_target_position(theta3)
        self._actuators['endEffector_motor'].set_target_position(theta1)
        post1 = int(self._actuators['base_motor'].get_position() * 100) / 100
        post2 = int(self._actuators['first_motor'].get_position() * 100) / 100
        post3 = int(self._actuators['second_motor'].get_position() * 100) / 100

        while not(post1 - 0.05 <= t1 <= post1+0.05) or not(post2 - 0.05 <= t2 <= post2+0.05) or not(post3 - 0.05 <= t3 <= post3+0.05):
            post1 = int(self._actuators['base_motor'].get_position() * 100) / 100
            post2 = int(self._actuators['first_motor'].get_position() * 100) / 100
            post3 = int(self._actuators['second_motor'].get_position() * 100) / 100
        time.sleep(0.05)
        self._state.Moving = False


    def enable_suction(self):
        self._state.pickedUp = True
        self.setIntegerSignal(self._suctionHandler, 1)

    def disableSuction(self):
        self.setIntegerSignal(self._suctionHandler, 0)

    def get_motors_theta(self, x: float, y: float, z: float):
        """"
        Inverse kinematics function for Uarm
        :param x: x position of the destination
        :param y: y position of the destination
        :param z: z position of the destination
        :return: tuple(theta1,theta2,theta3) Rotation of each motor to bring the end effector in the desired position
        """

        arcos = np.arccos
        atan = np.arctan2
        sqrt = np.sqrt
        # arms dimensions and end effector displacement

        a1, a2, a3, displacement, theta_displacement = self.get_sizes()
        dx, dy, dz = displacement
        nx = x
        ny = y
        nz = z + dz

        theta1 = atan(ny, nx)
        r1 = sqrt(ny ** 2 + nx ** 2) + dy
        FOR = 42 * np.pi / 180
        FOL = 25 * np.pi / 180
        s2h2 = r1 ** 2 + nz ** 2
        angleA = arcos((a2 ** 2 + a3 ** 2 - s2h2) / (2 * a2 * a3))
        angleB = np.arctan2(nz, r1)
        angleC = arcos((a2 ** 2 + s2h2 - a3 ** 2) / (2 * a2 * np.sqrt(s2h2)))
        theta3 = np.pi - angleA - angleB - angleC + FOR
        theta2 = angleB + angleC + FOL

        return abs(theta1), theta2, theta3


    # region Percept

    def getState(self):
        s = {}

        s["tracking"]= self._state.Moving
        s["holding"]= self._state.isHolding
        s["last_held"]= self._state.lastHeld
        s["over_home"] = self._state.overHome
        return s

    def get_percepts(self):
        """
        Reads the top and front Vision Sensors.
        :return array containing, for each detected blob
                (color, x_position, y_position, width, height):
        """

        # value gathered from the filters
        codeTop, stateTop, imageTop = self._sensors["cameraTop"].read()
        codeFront, stateFront, imageFront = self._sensors["cameraFront"].read()
        img = self._sensors["cameraTop"].raw_image()
        rawTop = np.array(img, dtype=np.uint8)
        rawTop.resize((128, 128, 3))

        rawFront = np.array(self._sensors["cameraFront"].raw_image(), dtype=np.uint8)
        rawFront.resize((128, 128, 3))
        object_list = self.readVisionData(imageTop, imageFront, rawTop, rawFront)
        print("OBJECT LIST: {}".format(object_list))
        self.set_state_env(object_list)
        print("------self.state.blockenv: {}".format(self._state.block_env))
        self._state.table_env = self.getFreeSlots(object_list, self._table_slots)
        self._state.shared_env = self.getFreeSlots(object_list, self._shared_slots)
        return object_list

    def readVisionData(self, imageTop, imageFront, rawTop, rawFront):
        blob_countTop = int(imageTop[1][0])

        blob_countFront = int(imageFront[1][0])

        vCntTop = int(imageTop[1][1])
        vCntFront = int(imageFront[1][1])
        blob_info_top = imageTop[1]
        blob_info_front = imageFront[1]

        RAD_2 = np.sqrt(2)

        object_list = {}
        cubes_top = {}

        for i in range(0, blob_countTop):
            orientation = blob_info_top[3 + (vCntTop * i)]
            posx = blob_info_top[3 + (vCntTop * i) + 1]
            posy = blob_info_top[3 + (vCntTop * i) + 2]
            width = blob_info_top[3 + (vCntTop * i) + 3]
            height = blob_info_top[3 + (vCntTop * i) + 4]

            if width >= 0.13:
                if height >= 0.13:
                    border_color = f.getColorName(*f.readImagesColor(rawTop, posy + height / 2 - 1, posx + width / 2 - 1))
                    if border_color == "grey":
                        newx = posx - width / 4
                        newy = posy + height / 2 - width / 4
                        cubes_top.update(f.assignCoordinateToColor(rawTop, newx, newy))
                        newx = posx + width / 4
                        newy = posy - height / 2 + width / 4
                        cubes_top.update(f.assignCoordinateToColor(rawTop, newx, newy))
                else:
                    if orientation < -0.1:
                        newx = posx - height / (2 * RAD_2)
                        newy = posy + width / (4 * RAD_2)
                        cubes_top.update(f.assignCoordinateToColor(rawTop, newx, newy))
                        newx = posx + height / (2 * RAD_2)
                        newy = posy - width / (4 * RAD_2)
                        cubes_top.update(f.assignCoordinateToColor(rawTop, newx, newy))
                    else:
                        newx = posx - width / 4
                        cubes_top.update(f.assignCoordinateToColor(rawTop, newx, posy))
                        newx = posx + width / 4
                        cubes_top.update(f.assignCoordinateToColor(rawTop, newx, posy))
            else:
                if height >= 0.13:
                    if orientation < -0.1:
                        newx = posx - width / (2 * RAD_2)
                        newy = posy - height / (4 * RAD_2)
                        cubes_top.update(f.assignCoordinateToColor(rawTop, newx, newy))
                        newx = posx + width / (2 * RAD_2)
                        newy = posy + height / (4 * RAD_2)
                        cubes_top.update(f.assignCoordinateToColor(rawTop, newx, newy))

                    else:
                        newy = posy + height / 4
                        cubes_top.update(f.assignCoordinateToColor(rawTop, posx, newy))
                        newy = posy - height / 4
                        cubes_top.update(f.assignCoordinateToColor(rawTop, posx, newy))
                else:
                    cubes_top.update(f.assignCoordinateToColor(rawTop, posx, posy))
        for i in range(0, blob_countFront):
            posx = blob_info_front[2 + (vCntFront * i) + 2]
            posy = blob_info_front[2 + (vCntFront * i) + 3]
            width = blob_info_front[2 + (vCntFront * i) + 4]
            height = blob_info_front[2 + (vCntFront * i) + 5]
            levels = round(height / width)


            if levels == 1:
                color = f.getColorName(*f.readImagesColor(rawFront, posy, posx))
                x, y = cubes_top[color]
                object_list.update({color: (x, y, 1)})
            else:
                first_block_z = posy + (height * (levels - 1)) / (2 * levels)
                color = f.getColorName(*f.readImagesColor(rawFront, first_block_z, posx))
                x, y = cubes_top[color]
                for j in range(0, levels):
                    color = f.getColorName(*f.readImagesColor(rawFront, first_block_z - ((height / levels) * j), posx))
                    object_list.update({color: (x, y, levels - j)})
        print("VISION: {}".format(object_list))
        return object_list
    # endregion Percepts

    def getFreeSlots(self, object_list: dict, slots):
        free_slots = []
        for x, y in slots:
            found = False
            for color, (px, py, pz) in object_list.items():
                if found:
                    break
                if x + 10 >= px >= x - 10 and y + 10 >= py >= y - 10:
                    found = True
            if not found:
                free_slots.append((x, y))
        return free_slots

    def process_commands(self, cmd):
        if cmd is not None:
            self.invoke(cmd['cmd'], cmd['args'])

    def invoke(self, cmd, args):
        print('invoke', cmd, args)
        if cmd != 'illegal_command':
            try:
                getattr(self.__class__, cmd)(self, *args)
            except AttributeError:
                raise NotImplementedError("Class `{}` does not implement `{}`".format(self.__class__.__name__, cmd))

    def setIntegerSignal(self, name: str, value: int):
        v.simxSetIntegerSignal(self._api._id, name, value,v.simx_opmode_oneshot)


class RobotTask(threading.Thread):
    def __init__(self, robot: RobotModel, cmd):
        threading.Thread.__init__(self)
        self.cmd = cmd
        self.robot = robot

    def run(self):
        self.robot.process_commands(self.cmd)

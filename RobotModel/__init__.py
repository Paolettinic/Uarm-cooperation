from pyrep import VRep
from pyrep.vrep import vrep as v
import RobotControl
import numpy as np
import threading
import functions as f


class RobotState:
    def __init__(self):
        self.block_env = []
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
        self._homePosition = (530 / 2, 0, 130) if self._name == "uarmR" else (-530 / 2, 0, 130)
        self._cameraResX = 128
        self._cameraResY = 128
        self._table_slots = []
        self._shared_slots = []

        if self._name == "uarmL":
            self._table_slots = [
                (0, 225),
                (-74, 225),
                (-153, 225),
                (-223, 225)
            ]
            self._shared_slots = [
                (74, 225),
                (149, 225),
                (223, 225),
            ]
        else:
            self._table_slots = [
                (0, 225),
                (74, 225),
                (149, 225),
                (223, 225),
            ]
            self._shared_slots = [
                (-74, 225),
                (-153, 225),
                (-223, 225)
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
        b_color = f.index_to_color(block)
        if table == 'shared':
            x, y, z = self._state.shared_env[b_color]
        else:
            x, y, z = self._state.block_env[b_color]
        scheme = [0, -10, 61, 115]
        self.place_end((int(x), int(y), 130))
        self.place_end((int(x), int(y), scheme[z]))
        self.enable_suction()
        self.set_holding(block)
        self.place_end((int(x), int(y), 130))
        self.go_home()
        self._controller.process_percepts()



    def put_on_block(self, block, table):
        b_color = f.index_to_color(block)
        if table == 'shared':
            x, y, z = self._state.shared_env[b_color]
            self.set_over_home(False)
        else:
            self.set_over_home(True)
            x, y, z = self._state.block_env[b_color]
        scheme = [0, -11, 61, 115]
        self.place_end((int(x), int(y), 180))
        self.place_end((int(x), int(y), scheme[z]+50))
        self.disableSuction()
        self.place_end((int(x), int(y), 180))
        self.go_home()
        self.set_tracking(False)
        self.set_last_held(self.is_holding())
        self.set_holding(0)
        self._controller.process_percepts()

    def put_on_table(self, table):
        if table == 'shared':
            self.set_over_home(False)
            x, y = self._state.shared_env.pop(0)  # Use the first free slot and removes it, as it will no longer be free
        else:
            self.set_over_home(True)
            x, y = self._state.table_env.pop(0)  # Use the first free slot and removes it, as it will no longer be free
        self.place_end((x, y, 130))
        self.place_end((x, y, 0))
        self.disableSuction()
        self.place_end((x, y, 130))
        self.go_home()
        self.set_last_held(self.is_holding())
        self.set_holding(0)
        self.set_over_home(True)
        self._controller.process_percepts()

    # endregion Teleor

    def place_end(self, coordinates: tuple):
        x, y, z = coordinates
        print(coordinates)
        theta = self.get_motors_theta(x, y, z)
        self._state.Moving = True
        self.rotate_motors(*theta)

    def rotate_motors(self, theta1, theta2, theta3):
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

        while t1 != post1 or t2 != post2 or t3 != post3:
            post1 = int(self._actuators['base_motor'].get_position() * 100) / 100
            post2 = int(self._actuators['first_motor'].get_position() * 100) / 100
            post3 = int(self._actuators['second_motor'].get_position() * 100) / 100

        #print("FINISHED MOVING")
        self._state.Moving = False

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

        # print("theta1 ", abs(theta1), "| theta2 ", theta2, "| theta3 ", theta3)

        return abs(theta1), theta2, theta3

    def enable_suction(self):
        self._state.pickedUp = True
        self.setIntegerSignal(self._suctionHandler, 1)

    def disableSuction(self):
        self.setIntegerSignal(self._suctionHandler, 0)

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
        self._state.block_env = object_list
        self._state.table_env = self.getFreeSlots(object_list, self._table_slots)
        self._state.shared_env = self.getFreeSlots(object_list, self._shared_slots)
        return object_list

    def readVisionData(self, imageTop, imageFront, rawTop, rawFront):
        blob_countTop = int(imageTop[1][0])
        # print(blob_countTop)

        blob_countFront = int(imageFront[1][0])
        # print(blob_countFront)
        vCntTop = int(imageTop[1][1])
        vCntFront = int(imageFront[1][1])
        blob_info_top = imageTop[1]
        blob_info_front = imageFront[1]

        RAD_2 = np.sqrt(2)

        object_list = {}
        cubes_top = {}
        cubes = []

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
        # print(cubes_top)
        for i in range(0, blob_countFront):
            posx = blob_info_front[3 + (vCntFront * i) + 1]
            posy = blob_info_front[3 + (vCntFront * i) + 2]
            width = blob_info_front[3 + (vCntFront * i) + 3]
            height = blob_info_front[3 + (vCntFront * i) + 4]
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
        # print(object_list)
        return object_list

    def getFreeSlots(self, object_list: dict, slots):
        free_slots = []
        for x, y in slots:
            found = False
            for color, (px, py, pz) in object_list.items():
                if found:
                    break
                if x + 5 >= px >= x - 5 and y + 5 >= py >= y - 5:
                    found = True
            if not found:
                free_slots.append((x, y))
        return free_slots

    def process_commands(self, cmd):
        print(cmd)
        if cmd is not None:
            self.invoke(cmd['cmd'], cmd['args'])

    def invoke(self, cmd, args):
        print(self._state.block_env)
        print(self._name)
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

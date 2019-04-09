from RobotModel import Uarm, VRep, RobotTask
import pedroclient
import queue
import threading
import time
import numpy as np
import functions as f


class Control(object):
    def __init__(self, host, port, sleep_time):
        self._host = host
        self._port = port
        self._sleep_time = sleep_time
        self._api = VRep.connect(self._host, self._port)
    def run(self):
        with self._api as api:
            robot_left, robot_right = self.make_robot(api)
            env_left = robot_left.get_percepts()
            env_right = robot_right.get_percepts()
            j_env = f.getMergedVision(env_left,env_right)
            self.process_initialize(j_env)
            while True:
                print(self.get_commands())
                #TODO: check if is cmd is One arm move o Two:
                # in the first case you just pass the command, otherwise you create two RobotTask and start them


    def make_robot(self, api) -> tuple:
        return (None,None)

    def process_initialize(self,block_percept,arm_percept):
        pass

    def process_percepts(self, block_percept,arm_percept):
        pass

    def get_commands(self):
        pass


# Handling messages from the TR program
class MessageThread(threading.Thread):
    def __init__(self, client, q):
        self.running = True
        self.client = client
        self.queue = q
        threading.Thread.__init__(self)
        self.daemon = True

    def run(self):
        while self.running:
            p2pmsg = self.client.get_term()[0]
            self.queue.put(p2pmsg)

    def stop(self):
        self.running = False

# TODO: Implement PedroControl

class PedroControl(Control):
    def __init__(self, host='127.0.0.1', port=19997, sleep_time=1.0):
        super().__init__(host, port, sleep_time)
        self.client = pedroclient.PedroClient()
        self.client.register("vrep_pedro")
        self.queue = queue.Queue(0)
        self.message_thread = MessageThread(self.client, self.queue)
        self.message_thread.start()
        self._tr_client_addr = 0
        p2pmsg = self.queue.get()
        self.percepts_addr = p2pmsg.args[1]

    def process_percepts(self, block_percepts, arm_percepts): #TODO: pass arm percept and add to msg
        msg = []

        for arm,holding in arm_percepts.items():
            if not holding:
                msg.append('f_(holding({0}, 1'.format(arm))
            else:
                msg.append('r_(holding({0}, {1}'.format(arm,holding))

        for block,on in block_percepts.items():
            if not on in ["table1","table2","shared"]:
                msg.append('r_(on("{0}", "{1}")'.format(f.set_index_color(block),f.set_index_color(on)))
            else:
                msg.append('r_(on("{0}", "{1}"'.format(f.set_index_color(block),on))


        self.send_percept('['+','.join(msg)+']')

    def send_percept(self, percepts_string):
        print("send_percept", str(self._tr_client_addr), percepts_string)
        if self.client.p2p(self._tr_client_addr, percepts_string) == 0:
            print("Error", percepts_string)


    def get_commands(self):
        cmds = []
        while not self.queue.empty():
            p2pmsg = self.queue.get()
            msg = p2pmsg.args[2]
            actions = msg.args[0]
            for a in actions.toList():
                cmds.append(self.action_to_command(a))
        return cmds

    def process_initialize(self,block_percept,arm_percept):
        # Block unitil message arrives
        p2pmsg = self.queue.get()
        print(p2pmsg)
        message = p2pmsg.args[2]
        if str(message) == 'initialise_':
            # get the sender address
            percepts_addr = p2pmsg.args[1]
            print("percepts_addr", str(percepts_addr))
            self.set_client(percepts_addr)
            self.send_percept(self.process_percepts(block_percept,arm_percept))
        else:
            print("Didn't get initialise_ message")

    def set_client(self, addr):
        self._tr_client_addr = addr

    def make_robot(self, api) -> (Uarm, Uarm):
        return Uarm('uarmL', api), Uarm('uarmR', api)

    def action_to_command(self, a):
        cmd_type = a.functor.val
        cmd = a.args[0]
        if cmd_type == 'stop_':
            return {'cmd': 'move_forward', 'args': [0.0]}
        #else return {'cmd':'action', 'args':[]}

    def pickup(self,block,arm):
        pass
    def unpile(self,block):
        pass
    def placeAoverB(self,block,block_or_tower):
        pass


def pedro_control():
    '''
        Out of process robot control by a teleor AI
    :return:
    '''
    vrep_pedro = PedroControl()
    # wait for and process initialize_ message
    #vrep_pedro.process_initialize()
    vrep_pedro.run()

class DemoControl(Control):

    def __init__(self, host='127.0.0.1', port=19997, sleep_time=2):
        super().__init__(host, port, sleep_time)
        self.i = 0


    def make_robot(self, api)-> (Uarm,Uarm):
        return Uarm('uarmL', api), Uarm('uarmR',api)

    def process_percepts(self, object_percepted,arm_status):
        for i in range((object_percepted)):
            print (i)

    def get_commands(self):
        return [{'cmd': 'placeEnd', 'args': [(-74,231,50)]}]

    def process_initialize(self, initial_env,arm_status):
         return [{'cmd': 'placeEnd', 'args': [(0, 100, 100)]}]




def demo_control():
    vrep_demo = DemoControl()
    #vrep_demo.process_initialize()
    #time.sleep(1)
    vrep_demo.run()
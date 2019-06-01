from RobotModel import Uarm, VRep, RobotTask
import pedroclient
import queue
import threading
import time
import functions as f


class Control(object):
    def __init__(self, host, port, sleep_time):
        self._host = host
        self._port = port
        self._sleep_time = sleep_time
        try:
            self._api = VRep.connect(self._host, self._port)
        except:
            print('V-REP not responding')
            exit(-1)
        self.robotL, self.robotR = self.make_robot(self._api)
        self._api.simulation.start()

    def run(self):
        mv = f.getMergedVision(
            self.robotL.get_percepts(),
            self.robotR.get_percepts()
        )
        while True:
            pass


    def make_robot(self, api) -> tuple:
        return None, None

    def process_initialize(self):
        pass

    def process_percepts(self):
        pass

    def get_commands(self):
        pass

    def send_percept(self, percepts_string):
        pass


# Handling messages from the TR program
class MessageThread(threading.Thread):
    def __init__(self, client, q, pedrocontrol):
        self.running = True
        self.client = client
        self.queue = q
        self.pedrocontrol = pedrocontrol
        threading.Thread.__init__(self)
        self.daemon = True

    def run(self):
        while self.running:
            p2pmsg = self.client.get_term()[0]
            # print("-----------P2P MESSAGE_",p2pmsg)
            self.queue.put(p2pmsg)
            self.pedrocontrol.get_commands()

    def stop(self):
        self.running = False


class PedroControl(Control):
    def __init__(self, host='127.0.0.1', port=19997, sleep_time=1):
        super().__init__(host, port, sleep_time)
        self.client = pedroclient.PedroClient()
        self.client.register("vrep_pedro")
        self.queue = queue.Queue(0)
        self.message_thread = MessageThread(self.client, self.queue, self)
        self.message_thread.start()
        self._tr_client_addr = 0

    def make_robot(self, api,) -> (Uarm, Uarm):
        return Uarm('uarmL', api, self), Uarm('uarmR', api, self)

    def get_perc(self):
        block_perceptL = self.robotL.get_percepts()
        block_perceptR = self.robotR.get_percepts()
        merged_vision = f.getMergedVision(block_perceptL, block_perceptR)
        arm_statusL = self.robotL.getState()
        arm_statusR = self.robotR.getState()
        return merged_vision, arm_statusL, arm_statusR

    # RECEIVE
    def set_client(self, addr):
        self.percepts_addr = addr

    def process_initialize(self):
        # Block until message arrives
        self.robotL, self.robotR = self.make_robot(self._api)
        self.process_percepts()


    def action_to_command(self, a) -> RobotTask:
        cmd_type = a.functor.val
        cmd = a.args[0]
        command = ""
        if cmd.functor.val == "pickup":
            command = {'cmd': cmd.functor.val, 'args': [cmd.args[1].val, cmd.args[2].val]}
        if cmd.functor.val == "put_on_table":
            command = {'cmd': cmd.functor.val, 'args': [cmd.args[1].val]}
        if cmd.functor.val == "put_on_block":
            command = {'cmd': cmd.functor.val, 'args': [cmd.args[1].val, cmd.args[2].val]}
        print("FIRST ARG: {}".format(cmd.args[0]))
        if str(cmd.args[0]) == "uarmL":
            return RobotTask(self.robotL,command)
        else:
            return RobotTask(self.robotR, command)

    def get_commands(self):
        p2pmsg = self.queue.get()
        # print("P2PMSG=", p2pmsg)
        msg = p2pmsg.args[2]
        actions = msg
        if str(msg) == 'initialise_':
            print("initialized")
            percepts_addr = p2pmsg.args[1]
            self.set_client(percepts_addr)
            self.process_initialize()
        if msg.get_type() == pedroclient.PObject.listtype:
            for a in actions.toList():
                if a.functor.val == 'start_':
                    self.action_to_command(a).start()
                else:
                    pass


    # SEND
    def process_percepts(self):
        msg = []
        merged_vision, arm_statusL, arm_statusR = self.get_perc()
        if not arm_statusL['holding'] == 0:
            msg.append('r_(holding(uarmL, {0}))'.format(arm_statusL['holding']))
        else:
            msg.append('f_(holding(uarmL, {0}))'.format(arm_statusL["last_held"]))

        if not arm_statusR['holding'] == 0:
            msg.append('r_(holding(uarmR, {0}))'.format(arm_statusR['holding']))
        else:
            msg.append('f_(holding(uarmR, {0}))'.format(arm_statusR["last_held"]))

        for on, under in merged_vision.items():
            if not under in ['table1', 'shared', 'table2']:
                msg.append('r_(on({0},{1}))'.format(on, under))
            else:
                msg.append('r_(on_table({0},{1}))'.format(on, under))

        if not arm_statusL['over_home']:
            msg.append('f_(over_home(uarmL))')
        else:
            msg.append('r_(over_home(uarmL))')

        if not arm_statusR['over_home']:
            msg.append('f_(over_home(uarmR))')
        else:
            msg.append('r_(over_home(uarmR))')

        if not arm_statusL['tracking']:
            msg.append('f_(tracking(uarmL))')
        else:
            msg.append('r_(tracking(uarmL))')
        if not arm_statusR['tracking']:
            msg.append('f_(tracking(uarmR))')
        else:
            msg.append('r_(tracking(uarmR))')
        print(msg)
        self.send_percept(msg)

    def send_percept(self, percept):
        percepts_string = '[' + ','.join(percept) + ']'
        print("send_percept", str(self.percepts_addr), percepts_string)
        if self.client.p2p(self.percepts_addr, percepts_string) == 0:
            print("Error", percepts_string)

    # def pickup(self, arm, block, table):
    #     if arm == 'uarm_L':
    #         rt = RobotTask(self.robotL,cmd)





def pedro_control():
    '''
        Out of process robot control by a teleor AI
    :return:
    '''
    vrep_pedro = PedroControl()

    vrep_pedro.run()

# class DemoControl(Control):
#
#     def __init__(self, host='127.0.0.1', port=19997, sleep_time=2):
#         super().__init__(host, port, sleep_time)
#         self.i = 0
#
#
#     def make_robot(self, api)-> (Uarm,Uarm):
#         return Uarm('uarmL', api),Uarm('uarmR', api)
#
#     def process_percepts(self, object_percepted,arm_status):
#         for i in range((object_percepted)):
#             print (i)
#
#     def get_commands(self):
#         return [{'cmd': 'placeEnd', 'args': [(-74,231,50)]}]
#
#     def process_initialize(self):
#          return [{'cmd': 'placeEnd', 'args': [(0, 100, 100)]}]
#
#
#
#
# def demo_control():
#     vrep_demo = DemoControl()
#     #vrep_demo.process_initialize()
#     #time.sleep(1)
#     vrep_demo.run()

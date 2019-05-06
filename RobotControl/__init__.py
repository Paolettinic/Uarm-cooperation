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
        self.robotL,self.robotR = None,None
        self._api.simulation.start()

    def run(self):

        while True:
            self.process_percepts(
                f.getMergedVision(
                    self.robotL.get_percepts(),
                    self.robotR.get_percepts()
                ),
                self.robotL.getState(),
                self.robotR.getState()
            )
            cmd = self.get_commands()
            print("COMMAND ________________",cmd)
            task_left = RobotTask(self.robotL, cmd)

            task_right = RobotTask(self.robotR, cmd)
            task_left.start()
            task_right.start()
            task_right.join()
            task_left.join()
            time.sleep(0)


    def make_robot(self, api) -> tuple:
        return None,None


    def process_initialize(self):
        pass

    def process_percepts(self, block_percept, arm_statusL, arm_statusR):
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
            #print("-----------P2P MESSAGE_",p2pmsg)
            self.queue.put(p2pmsg)

    def stop(self):
        self.running = False

class PedroControl(Control):
    def __init__(self, host='127.0.0.1', port=19997, sleep_time=1):
        super().__init__(host, port, sleep_time)
        self.client = pedroclient.PedroClient()
        self.client.register("vrep_pedro")
        self.queue = queue.Queue(0)
        self.message_thread = MessageThread(self.client, self.queue)
        self.message_thread.start()
        self._tr_client_addr = 0


    def process_percepts(self, block_percepts, arm_statusL, arm_statusR):
        msg = []
        if not arm_statusL['holding'] == 0:
            msg.append('r_(holding(uarmL, {0}))'.format(arm_statusL['holding']))
        else:
            msg.append('f_(holding(uarmL, {0}))'.format(arm_statusL["last_held"]))

        if not arm_statusR['holding'] == 0:
            msg.append('r_(holding(uarmR, {0}))'.format(arm_statusR['holding']))
        else:
            msg.append('f_(holding(uarmR, {0}))'.format(arm_statusR["last_held"]))

        for on,under in block_percepts.items():
            if not under in ['table1','shared','table2']:
                msg.append('r_(on({0},{1}))'.format(on,under))
            else:
                msg.append('r_(on_table({0},{1}))'.format(on,under))

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
            msg.append('r_(tracking(uarmL,)')
        if not arm_statusR['tracking']:
            msg.append('f_(tracking(uarmR))')
        else:
            msg.append('r_(tracking(uarmR,)')

        self.send_percept('['+','.join(msg)+']')


    def send_percept(self, percepts_string):
            print("send_percept", str(self.percepts_addr), percepts_string)
            if self.client.p2p(self.percepts_addr, percepts_string) == 0:
                print("Error", percepts_string)


    def get_commands(self):
        cmds = []
        while not self.queue.empty():
            p2pmsg = self.queue.get()
            print("P2PMSG=",p2pmsg)
            msg = p2pmsg.args[2]
            actions = msg
            if msg.get_type() == pedroclient.PObject.listtype :
                for a in actions.toList():
                    cmds.append(self.action_to_command(a))
                return cmds
            return []

    def process_initialize(self):
        # Block unitil message arrives
        self.robotL,self.robotR = self.make_robot(self._api)
        block_perceptL = self.robotL.get_percepts()
        arm_perceptL = self.robotL.is_holding()

        block_perceptR = self.robotR.get_percepts()
        arm_perceptR = self.robotR.is_holding()

        merged_vision = f.getMergedVision(block_perceptL,block_perceptR)

        arm_statusL = self.robotL.getState()
        arm_statusR = self.robotR.getState()

        p2pmsg = self.queue.get()
        print(p2pmsg)
        message = p2pmsg.args[2]
        if str(message) == 'initialise_':
            percepts_addr = p2pmsg.args[1]
            print("percepts_addr", str(percepts_addr))
            self.set_client(percepts_addr)
            self.process_percepts(merged_vision, arm_statusL, arm_statusR)
        else:
            print("Didn't get the initialize message")

    def set_client(self, addr):
        self.percepts_addr = addr

    def make_robot(self, api) -> (Uarm,Uarm):
        return Uarm('uarmL', api),Uarm('uarmR',api)

    def action_to_command(self, a):
        cmd_type = a.functor.val
        cmd = a.args[0]
        if cmd_type == 'start_':
            print("------------------------START CMD:\t\t\t",cmd.functor.val," | ",cmd.args[0])
            if cmd.functor.val == "pickup":
                return {'cmd': cmd.functor.val, 'args': [cmd.args[0].val,cmd.args[1].val,cmd.args[2].val]}
            if cmd.functor.val == "put_on_table":
                print("PUT ON TABLE")
                return {'cmd': "put_on_table", 'args': [cmd.args[0].val,cmd.args[1].val]}
            if cmd.functor.val == "put_on_block":
                print("PUT ON TABLE")
                return {'cmd': "put_on_block", 'args': [cmd.args[0].val,cmd.args[1].val,cmd.args[2].val]}
        else:
            print("--------------------------STOP CMD:\t\t\t", cmd)

        #else return {'cmd':'action', 'args':[]}



def pedro_control():
    '''
        Out of process robot control by a teleor AI
    :return:
    '''
    vrep_pedro = PedroControl()

    # wait for and process initialize_ message
    vrep_pedro.process_initialize()
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


from pedroclient import *

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


class Vrep_Pedro(object):

    def __init__(self, vrep_client_id):
        self.vrep_client_id = vrep_client_id
        self.tr_client_addr = None
        self.client = PedroClient()
        # register vrep_pedro as the name of this process with Pedro
        self.client.register("robot_sim")
        self.queue = queue.Queue(0)
        self.message_thread = MessageThread(self.client, self.queue)
        self.message_thread.start()
        self.set_client("['127.0.0.1']")

    # def methods for sensing and acting to the robot in the simulator



    def set_client(self, addr):
        self.tr_client_addr = addr

    def send_percept(self, percepts_string):
        print("send_percept", str(self.tr_client_addr), percepts_string)
        if self.client.p2p(self.tr_client_addr, percepts_string) == 0:
            print("Error", percepts_string)

    def exit(self):
        self.message_thread.stop()
        self.client.p2p("messages:" + self.tr_client_addr, "quiting")

    def process_initialize(self):
        # Block unitil message arrives
        print('listening start signal..')
        p2pmsg = self.queue.get()
        print(p2pmsg)
        message = p2pmsg.args[2]
        if str(message) == 'initialise_':
            # get the sender address
            percepts_addr = p2pmsg.args[1]
            print("percepts_addr", str(percepts_addr))
            self.set_client(percepts_addr)
            # VREP code goes here so the visualization can
            # send back any initial percepts (iniital state)
            # create a string representing a list of initial percepts
            # say init_percepts and call
            # self.parent.send_percept(init_percepts)
            init_percepts = '[]'
            self.send_percept(init_percepts)
        else:
            print("Didn't get initialise_ message")

    def process_controls(self):
        while not self.queue.empty():
            p2pmsg = self.queue.get()
            msg = p2pmsg.args[2]
            print("process_controls message: ", str(msg))
            if not msg.is_pstruct() or msg.functor.val != 'controls':
                print("CONTROLS: ", str(msg))
                assert False
            actions = msg.args[0]
            if not actions.is_plist():
                print("CONTROLS: ", str(actions))
                assert False
            for a in actions.toList():
                self.process_action(a)

    def process_action(self, message):
        if not message.is_pstruct():
            return
        functor = message.functor
        if not functor.is_patom():
            return
        cmd_type = functor.val
        cmd = message.args[0]
        if not cmd.is_pstruct():
            return

        #if cmd_type == 'value'
        #   self.action()

        '''if cmd_type == 'stop_':
            if cmd.functor.val == 'move' and cmd.arity() == 2:
                self.stop_move()
        elif cmd_type in ['start_', 'mod_']:
            if cmd.functor.val == 'move' and cmd.arity() == 1:
                speed = cmd.args[0].val
                self.move_forward(speed)
            if cmd.functor.val == 'turn_left' and cmd.arity() == 1:
                speed = cmd.args[0].val
                self.turn_left(speed)
            if cmd.functor.val == 'turn_right' and cmd.arity() == 1:
                speed = cmd.args[0].val
                self.turn_right(speed)'''

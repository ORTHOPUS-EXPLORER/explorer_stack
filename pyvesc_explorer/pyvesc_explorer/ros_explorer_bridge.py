# DO NOT REORDER, nest_asyncio must be imported and apply() called before importing syncio
import nest_asyncio
nest_asyncio.apply()
import asyncio

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from control_msgs.msg import DynamicJointState, InterfaceValue
from pyvesc_explorer_msgs.msg import JointsCommands,JointPrint
from rclpy.action import ActionServer
from rclpy.qos import QoSPresetProfiles

import pyvesc
import asyncio

import time
import random

class ROSExplorerPub(Node):

    pos_meas = [0.0]*6
    pos_ref = [0.0]*6

    def pub_ref(self,joint_ref=[None]*6):
        msg = DynamicJointState()
        i = 0
        if len(joint_ref) != 6:
            joint_ref = random.sample(range(-180,180),6)
        for i in range(0,6):
            # Publish !
            self.pos_ref[i] = joint_ref[i] if joint_ref[i] is not None else (self.pos_meas[i] + random.uniform(-5,5))
            msg.joint_names.append("joint_"+str(1+i))
            ifv = InterfaceValue()
            ifv.interface_names = ["position"]     # FIXME: Support more interfaces
            ifv.values = [self.pos_ref[i]]
            msg.interface_values.append(ifv)
            i += 1
        #print(msg)
        self.pos_ref_pub.publish(msg)

    def pub_cmd(self, cmds=[]*6):
        msg = JointsCommands()
        i = 0
        if len(cmds) != 6:
            cmds = ["ping"]*6
        for i in range(0,6):
            # Publish !
            msg.joint_names.append("joint_"+str(1+i))
            msg.commands.append(cmds[i])
        #print(msg)
        self.cmds_pub.publish(msg)

    def pos_meas_cb(self, data):
        for axis in range(0,6):
            name = "joint_"+str(1+axis)
            i = 0
            for j in data.joint_names:
                if j == name:
                    break
                i += 1
            if i >= len(data.joint_names):
                continue
            ifv = data.interface_values[i]
            j = 0
            for k in ifv.interface_names:
                if k == "position":     # FIXME: Support more interfaces
                    break
                j += 1
            if j >= len(ifv.interface_names):
                continue
            self.pos_meas[axis] = ifv.values[j]

    def texts_cb(self, data):
        print("<< [{:s}] MSG: '{:s}'".format(data.joint_name, data.text))

    def __init__(self):
        super().__init__('ros_explorer_pub')
        # ROS pub/sub
        self.pos_ref_pub  = self.create_publisher(DynamicJointState,"/explorer_ref", QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.pos_meas_sub = self.create_subscription(DynamicJointState,"/explorer_meas", self.pos_meas_cb, QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.texts_sub = self.create_subscription(JointPrint,"/explorer_print", self.texts_cb, QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.cmds_pub  = self.create_publisher(JointsCommands,"/explorer_cmd", QoSPresetProfiles.SYSTEM_DEFAULT.value)

class PyVESCExplorerBridge(Node):

    def pos_ref_cb(self,ref_data):
        #print(ref_data)
        for axis in range(0,6):
            name = "joint_"+str(1+axis)
            i = 0
            for j in ref_data.joint_names:
                if j == name:
                    break
                i += 1
            if i >= len(ref_data.joint_names):
                continue
            ifv = ref_data.interface_values[i]
            j = 0
            for k in ifv.interface_names:
                if k == "position":
                    break
                j += 1
            if j >= len(ifv.interface_names):
                continue
            pos_ref = ifv.values[j]
            pos_meas = self.mespos[axis]
            delta = pos_meas-pos_ref
            if abs(delta) < self.max_pos_setpoint_step or \
               abs(delta - 360) < self.max_pos_setpoint_step or \
               abs(delta + 360) < self.max_pos_setpoint_step:  #TODO: allow moving in the direction that reduces delta
                self.vesc[axis].set_pos(pos_ref)
                self.lastpossetpoint[axis] = pos_ref
            else:
                self.get_logger().info('Axis %d, did not send the command, setpoint too far away' %(axis))

    def cmds_cb(self, cmds):
        i = 0
        for v in self.vesc:
            for j in range(0,6):
                jname = "joint_"+str(1+i)
                if cmds.joint_names[j] == jname:
                    jcmd = cmds.commands[j]
                    #print(">> [{:d}][{:s}] CMD: '{:s}'".format(v.can_id,jname, jcmd))
                    v.text_cmd(jcmd)

            i += 1

    def vescPrintHdlr(self, txt, vesc_id=-1):
        i = 1
        for v in self.vesc:
            jname = "joint_"+str(i)
            if v.can_id == vesc_id:
                #print("<< [{:d}][{:s}] MSG: {}".format(vesc_id,jname,txt))
                msg = JointPrint()
                msg.joint_name = jname
                msg.text = txt
                self.texts.publish(msg)
                import time
                time.sleep(0.01)
            i +=1

    def __init__(self,can_port='vcan0',can_id=40):
        super().__init__('ros_explorer_bridge')
        self.declare_parameter('vesc_can_id', [41,42,43,44,46,45])
        self.declare_parameter('max_pos_setpoint_step', 10.0)
        self.declare_parameter('can_port', can_port)
        self.declare_parameter('can_id', can_id)
        self.vesc_can_id = self.get_parameter('vesc_can_id').get_parameter_value().integer_array_value
        can_port = self.get_parameter('can_port').get_parameter_value().string_value
        can_id = self.get_parameter('can_id').get_parameter_value().integer_value
        nvesc = len(self.vesc_can_id)
        self.max_pos_setpoint_step = self.get_parameter('max_pos_setpoint_step').get_parameter_value().double_value
        self.axes = list(range(1,nvesc+1))
        self.posnone = [0]*nvesc
        self.mespos = [0.0]*nvesc
        self.lastpossetpoint = [0.0]*nvesc
        # Setup ROS pub/sub
        self.pos_meas = self.create_publisher(DynamicJointState,"/explorer_meas", QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.pos_ref  = self.create_subscription(DynamicJointState,"/explorer_ref", self.pos_ref_cb, QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.texts = self.create_publisher(JointPrint,"/explorer_print", QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.cmds  = self.create_subscription(JointsCommands,"/explorer_cmd", self.cmds_cb, QoSPresetProfiles.SYSTEM_DEFAULT.value)

        # Start PyVESC comm
        can0_comm    = {'type': 'can', 'driver': 'socketcan', 'port': can_port,  'bitrate': 500000, 'can_id': can_id}
        params = []
        for id in self.vesc_can_id:
            params += [{'comm': can0_comm, 'can_id': id,  'has_sensor': False, 'start_heartbeat': False },]
        self.mvesc = pyvesc.MultiVESC(params)
        self.vesc = []
        for id in self.vesc_can_id:
            v = self.mvesc(id)
            v.printHandler = lambda txt,i=id: self.vescPrintHdlr(txt,vesc_id=i)
            if v is not None:
                self.vesc.append(self.mvesc(id))
            else:
                self.get_logger().error("VESC '{:d}' not found".format(id))
                exit(-1)
        # Start timer to refresh meas
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def getVESC(self):
        return self.mvesc

    def __del__(self):
        self.get_logger().info("Leaving 1/2. Set brake current")
        for v in self.vesc:
            v.set_brake_current(1.0)
            self.get_logger().debug('ID %d, Current position: %.2f' %(v.can_id, v.pid_pos))
        self.get_logger().info('Waiting 15 seconds')
        import time
        #time.sleep(15)
        self.get_logger().info("Leaving 2/2. Zero-ing currents")
        for v in self.vesc:
            v.set_current(0)

    def timer_callback(self):
        msg = DynamicJointState()
        i = 0
        for v in self.vesc:
            # Get data from PyVESC
            pid_pos = v.pid_pos
            # Filter and handle errors
            if pid_pos is not None:
                if pid_pos > 180:
                    pid_pos -= 360 #TODO: check, probably not always coherent
                self.mespos[i] = pid_pos

            # Publish !
            pos = self.mespos[i]
            err = (self.lastpossetpoint[i] - pos) if self.lastpossetpoint[i] is not None else 0.0
            msg.joint_names.append("joint_"+str(1+i))
            ifv = InterfaceValue()
            ifv.interface_names = ["position","error"]
            ifv.values = [pos, err]
            msg.interface_values.append(ifv)
            i += 1
        self.pos_meas.publish(msg)

import click
@click.command(context_settings=dict(
    ignore_unknown_options=True,
))
@click.option('-n','--non-interactive', is_flag=True, default=False, help='Run in non-interactive mode')
@click.option('-P','--can-port', default='vcan0', help='CAN Port')
@click.option('-i','--can-id', default=40, help='CAN Id')
@click.argument('ros_args', nargs=-1, type=click.UNPROCESSED)
def MainApp(non_interactive, can_port, can_id, ros_args=[]):
    rclpy.init(args=list(ros_args))
    node = PyVESCExplorerBridge(can_port, can_id)
    node_pub = None if non_interactive else ROSExplorerPub()

    async def ros_loop():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            if node_pub:
                rclpy.spin_once(node_pub, timeout_sec=0)
            await asyncio.sleep(1e-4)

    async def shell():
        if non_interactive:
            return
        from traitlets.config.loader import Config
        from IPython.terminal.embed import InteractiveShellEmbed
        ipcfg = Config()
        ipcfg.InteractiveShell.confirm_exit = False
        ipcfg.using = 'asyncio'
        ipshell = InteractiveShellEmbed(
            config=ipcfg,
            banner1='Explorer interactive shell'
        )

        vesc = node.getVESC()
        for c in vesc.controllers:
            v = c.get_firmware_version()
            print("[0x{:02X}] FW version: {:6s} - HW: {:15s} - UUID: 0x{:24X}".format(c.can_id if c.can_id is not None else 0, str(v),str(v.hw_name),v.uuid))

        def pub(*args,**kwargs):
            node_pub.pub_ref(*args,**kwargs)

        def cmd(*args,**kwargs):
            node_pub.pub_cmd(*args,**kwargs)

        ipshell()
        rclpy.shutdown()

    asyncio.run((await asyncio.gather( shell(), ros_loop()) for _ in '_').__anext__())


if __name__ == '__main__':
    MainApp()

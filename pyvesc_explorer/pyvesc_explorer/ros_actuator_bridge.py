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
import math

class ROSActuatorPub(Node):

    pos_meas = [0.0]*1
    pos_ref = [0.0]*1

    def pub_ref(self,joint_ref=[None]*1):
        msg = DynamicJointState()
        i = 0
        if len(joint_ref) != 1:
            joint_ref = random.sample(range(-180,180),1)
        for i in range(0,1):
            # Publish !
            self.pos_ref[i] = joint_ref[i] if i < len(joint_ref) and joint_ref[i] is not None else ((self.pos_meas[i] if i < len(self.pos_meas) else 0.0) + random.uniform(-5,5))
            msg.joint_names.append("joint_"+str(1+i))
            ifv = InterfaceValue()
            ifv.interface_names = ["position"]     # FIXME: Support more interfaces
            ifv.values = [self.pos_ref[i]]
            msg.interface_values.append(ifv)
            i += 1
        #print(msg)
        self.pos_ref_pub.publish(msg)

    def pub_cmd(self, cmds=[]*1):
        msg = JointsCommands()
        i = 0
        if len(cmds) != 1:
            cmds = ["ping"]*1
        for i in range(0,1):
            # Publish !
            msg.joint_names.append("joint_"+str(1+i))
            msg.commands.append(cmds[i])
        self.cmds_pub.publish(msg)

    def pos_meas_cb(self, data):
        for axis in range(0,1):
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
        super().__init__('ros_actuator_pub')
        # ROS pub/sub
        self.pos_ref_pub  = self.create_publisher(DynamicJointState,"/explorer_ref", QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.pos_meas_sub = self.create_subscription(DynamicJointState,"/explorer_meas", self.pos_meas_cb, QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.texts_sub = self.create_subscription(JointPrint,"/explorer_print", self.texts_cb, QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.cmds_pub  = self.create_publisher(JointsCommands,"/explorer_cmd", QoSPresetProfiles.SYSTEM_DEFAULT.value)

class PyVESCActuatorBridge(Node):

    vesc = []
    vesc_mappings = []

    def pos_ref_cb(self,ref_data):
        #print(ref_data)
        for m in self.vesc_mappings:
            v = m['vesc']
            for jit in m['joints']:
                i = 0
                for j in ref_data.joint_names:
                    if j == jit['name']:
                        break
                    i += 1
                # joint name not found
                if i >= len(ref_data.joint_names):
                    continue
                ifv = ref_data.interface_values[i]
                j = 0
                for k in ifv.interface_names:
                    if k in ["position","velocity","acceleration","effort"]:
                        break
                    j += 1
                # interface not found
                if j >= len(ifv.interface_names):
                    continue
                pos_ref = ifv.values[j]
                # Check for None and NaN
                if pos_ref is None or pos_ref != pos_ref:
                    self.get_logger().error("[{:s}] Did not send the command, setpoint is None or NaN".format(jit['name']))
                    continue
                if jit['type'] == 'motor':
                    pos_meas = jit['posmeas']
                    delta = pos_meas-pos_ref
                    if abs(delta) < self.max_pos_setpoint_step or \
                    abs(delta - math.radians(360)) < self.max_pos_setpoint_step or \
                    abs(delta + math.radians(360)) < self.max_pos_setpoint_step:  #TODO: allow moving in the direction that reduces delta
                        v.set_pos(math.degrees(pos_ref))
                        jit['posref'] = pos_ref
                    else:
                        self.get_logger().warning("[{:s}] Did not send the command, setpoint '{:f}' too far away from current meas '{:f}'".format(jit['name'],pos_ref,pos_meas))
                elif jit['type'] == 'servo':
                    v.set_servo(pos_ref)

    def cmds_cb(self, cmds):
        j = 0
        if len(cmds.commands) != len(cmds.joint_names):
            # FIXME! Throw errors
            return
        # For each command
        for jcmd in cmds.commands:
            jname = cmds.joint_names[j]
            j += 1
            # Let's try to find the matching joint in our mappings
            for m in self.vesc_mappings:
                v = m['vesc']
                if v is None:   # Can't send command to non-existing VESC
                    print("VESC is None")
                    continue
                for jit in m['joints']:
                    if jname == jit['name']:
                        #print(">> [{:d}][{:s}] CMD: '{:s}'".format(v.can_id,jname, jcmd))
                        v.text_cmd(jcmd)
                    break

    def vescPrintHdlr(self, txt, vesc_id=-1):
        for v in self.vesc_mappings:
            if v['can_id'] == vesc_id:
                jname = v['joints'][0]['name']
                #print("<< [{:d}][{:s}] MSG: {}".format(vesc_id,jname,txt))
                msg = JointPrint()
                msg.joint_name = jname
                msg.text = txt
                self.texts.publish(msg)
                # Required to read multiple prints
                import time
                time.sleep(0.01)

    def __init__(self,can_port='vcan0',can_id=40):
        super().__init__('ros_actuator_bridge')
        self.vesc_mappings =  []

        self.declare_parameter('max_pos_setpoint_step', 10.0)

        self.declare_parameter('can_port', can_port)
        can_port = self.get_parameter('can_port').get_parameter_value().string_value

        self.declare_parameter('can_id', can_id)
        can_id = self.get_parameter('can_id').get_parameter_value().integer_value

        self.declare_parameter('vesc_joints_can_ids',[41])
        can_ids = self.get_parameter('vesc_joints_can_ids').get_parameter_value().integer_array_value
        self.declare_parameter('vesc_joint_names',['joint'])
        joint_names = self.get_parameter('vesc_joint_names').get_parameter_value().string_array_value
        self.declare_parameter('vesc_joint_types',['motor'])
        joint_types = self.get_parameter('vesc_joint_types').get_parameter_value().string_array_value

        if len(can_ids) != len(joint_names) or len(can_ids) != len(joint_types):
            print("Invalid parameters")
            quit()

        for i in range(0,len(can_ids)):
            can_id = can_ids[i]
            joint_name = joint_names[i]
            joint_type = joint_types[i]
            m = None
            for it in self.vesc_mappings:
                if it['can_id'] == can_id:
                    m = it
                    break
            jointv = {
                    'name':joint_name,
                    'type': joint_type,
                    'posnone': 0.0,
                    'posmeas': None,
                    'posref': None,
                }
            if m is None:
                m = {'can_id':can_id,'vesc':None,'joints':[jointv]}
                self.vesc_mappings.append(m)
            else:
                j = None
                for jt in m['joints']:
                    if jt['name'] == joint_name:
                        j = jt
                        print("There can only be ONE {:s} for VESC {:d}".format(joint_name,can_id))
                        break
                    elif jt['type'] == joint_type:
                        j = jt
                        print("There can only be ONE {:s} joint for VESC {:d}".format(joint_type,can_id))
                        break
                if j is None:
                    m['joints'].append(jointv)

        self.max_pos_setpoint_step = math.radians(self.get_parameter('max_pos_setpoint_step').get_parameter_value().double_value)

        # Setup ROS pub/sub
        self.pos_meas = self.create_publisher(DynamicJointState,"/explorer_meas", QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.pos_ref  = self.create_subscription(DynamicJointState,"/explorer_ref", self.pos_ref_cb, QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.texts = self.create_publisher(JointPrint,"/explorer_print", QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.cmds  = self.create_subscription(JointsCommands,"/explorer_cmd", self.cmds_cb, QoSPresetProfiles.SYSTEM_DEFAULT.value)

        # Start PyVESC comm
        can0_comm    = {'type': 'can', 'driver': 'socketcan', 'port': can_port,  'bitrate': 500000, 'can_id': can_id}
        params = []
        for it in self.vesc_mappings:
            id = it['can_id']
            params += [{'comm': can0_comm, 'can_id': id,  'has_sensor': False, 'start_heartbeat': False },]
        self.mvesc = pyvesc.MultiVESC(params)
        self.vesc = []
        for it in self.vesc_mappings:
            id = it['can_id']
            v = self.mvesc(id)
            v.printHandler = lambda txt,i=id: self.vescPrintHdlr(txt,vesc_id=i)
            if v is not None:
                self.vesc.append(self.mvesc(id))
                it['vesc'] = v
            else:
                self.get_logger().error("VESC '{:d}' not found".format(id))
                exit(-1)
        # Start timer to refresh meas
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = DynamicJointState()
        for m in self.vesc_mappings:
            v = m['vesc']
            # Get data from PyVESC
            # Filter and handle errors
            pid_pos = math.radians(v.pid_pos)
            if pid_pos is not None:
                if pid_pos > math.radians(180):
                    pid_pos -= math.radians(360) #TODO: check, probably not always coherent
            # FIXME: Also publish velocity, accel, torque
            for t,v in [('motor',pid_pos)]:#,('servo':-1.0)]
                for jit in m['joints']:
                    if t == jit['type']:
                        #print(jit['name']+' read pos: '+str(v))
                        jit['posmeas'] = v if v is not None else jit['posnone']

            # Publish !
            for jit in m['joints']:
                if jit['type'] != 'motor':
                    continue
                pos = jit['posmeas']  if jit['posmeas'] is not None else jit['posnone']
                err = (jit['posref'] - pos) if jit['posref'] is not None else 0.0
                msg.joint_names.append(jit['name'])
                ifv = InterfaceValue()
                ifv.interface_names = ["position","error"]
                ifv.values = [pos, err]
                msg.interface_values.append(ifv)
        self.pos_meas.publish(msg)

    def getVESC(self):
        return self.mvesc

    def __del__(self):
        self.get_logger().info("Leaving 1/2. Set brake current")
        for m in self.vesc_mappings:
            v = m['vesc']
            if v is not None:
                v.set_brake_current(1.0)
                self.get_logger().debug('ID %d, Current position: %.2f' %(v.can_id, v.pid_pos))
        self.get_logger().info('Waiting 15 seconds')
        import time
        #time.sleep(15)
        self.get_logger().info("Leaving 2/2. Zero-ing currents")
        for m in self.vesc_mappings:
            v = m['vesc']
            if v is not None:
                v.set_current(0)

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
    node = PyVESCActuatorBridge(can_port, can_id)
    node_pub = None if non_interactive else ROSActuatorPub()

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
            banner1='Actuator interactive shell'
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

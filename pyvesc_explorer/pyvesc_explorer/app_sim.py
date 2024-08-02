#!/usr/bin/env python3

# sudo modprobe vcan
# sudo ip link add dev vcan0 type vcan
# sudo ip link set mtu 16 up dev vcan0
# ./app_sim.py

# DO NOT REORDER, nest_asyncio must be imported and apply() called before importing syncio
import nest_asyncio
nest_asyncio.apply()
import asyncio

import pyvesc
from pyvesc.comm.can import VESCCAN
from pyvesc.VESC.messages import *
from pyvesc.VESC.VESCDefinitions import *

import time
import random

class PyFakeVESC(pyvesc.VESC):
  def __init__(self, comm, can_id=None, has_sensor=False, start_heartbeat=True, timeout=0.05):
    super().__init__(comm, can_id, has_sensor, start_heartbeat, timeout)

    self.temp_fet   = 0.0
    self.temp_motor = 0.0
    self.current_in = 0.0
    self.pid_pos    = 0.0
    self.pid_ref    = 0.0

  def stream_status(self):
    b = b''
    b += int.to_bytes(int(self.temp_fet*10), byteorder='big', length=2, signed=True)   # temp_fet
    b += int.to_bytes(int(self.temp_motor*10), byteorder='big', length=2, signed=True) # temp_motor
    b += int.to_bytes(int(self.current_in*10), byteorder='big', length=2, signed=True) # current_in
    b += int.to_bytes(int(self.pid_pos*50), byteorder='big', length=2, signed=True)    # pid_pos
    self.comm.write_raw(((VESCCAN.CAN_PACKET_STATUS_4<<8) | self.can_id),list(b))

  def set_pos_from_pkt(self, data):
    v = data.pos
    self.pid_ref = v
    print("[0x{:2X}] Set_pos: {}".format(self.can_id,str(v)))


  def get_firmware_version(self, timeout=0):
    pkt = FWVersion(0x06,                                     # 'fw_version_major' / Byte
                      0x02,                                   # 'fw_version_minor' / Byte
                      "PyFakeVESC",                           # 'hw_name' / CString('utf8')
                      0xBABAD00DBABAD00D00000000+self.can_id, # 'uuid' / BytesInteger(12)
                      0x00,                                   # 'pairing_done' / Byte
                      0x00,                                   # 'fw_test_version_number' / Byte
                      VESCHWType.VESC_OTHER,                  # 'hw_type_vesc' / Byte
                      0x00                                    # 'custom_config' / Byte
                      )
    return pkt

  def term_cmd(self, scmd):
    print("<< CMD: "+str(scmd))
    sresp = ""
    match scmd:
      case 'help':
        sresp = "Available commands:\nping\nhelp"
      case 'ping':
        sresp = "pong"
      case 'hw_status':
        sresp = "PyFakeVESC is running!\n-H179"
      case _:
        sresp = "Invalid command. Try: help"
    if len(sresp):
      print(">> MSG: "+str(sresp))
      return printMsg(sresp)
    return None

from pyvesc.VESC.VESCPacket import VESCPktDecoder
class PyFakeVESCPktHandlers:
  @VESCPktDecoder.handler(VedderCmd.COMM_SET_POS)
  def cmd_handle_fw_version(comm, data, src_id):
    data = VESCMessage.unpack((VedderCmd.COMM_SET_POS&0xFF).to_bytes(length=1, byteorder="big")+data)

    comm.ctrlers[comm.id()].set_pos_from_pkt(data)
    return True

  @VESCPktDecoder.handler(VedderCmd.COMM_FW_VERSION)
  def cmd_handle_fw_version(comm, data, src_id):
    #logging.info("[PyVESCPktHandlers::COMM_FW_VERSION] Data from {:d}: {:s} ".format(src_id,str(data)))
    # Oh! Someone's asking us to anwser:
    if len(data) > 0:
      return None
    pkt = comm.ctrlers[comm.id()].get_firmware_version()
    comm.write(src_id, encode(pkt), 1)
    return True

  @VESCPktDecoder.handler(VedderCmd.COMM_TERMINAL_CMD)
  def cmd_handle_terminal_cmd(comm, data, src_id):
    #logging.info("[{:2d}][PyVESCPktHandlers::COMM_TERMINAL_CMD] Data from {:d}: {:s} ".format(comm.id(), src_id, str(data)))
    #scmd = data.decode("utf-8")
    pkt = VESCMessage.unpack((VedderCmd.COMM_TERMINAL_CMD&0xFF).to_bytes(length=1, byteorder="big")+data)
    rpkt = comm.ctrlers[comm.id()].term_cmd(pkt.cmd)
    if rpkt is not None:
      comm.write(src_id, encode(rpkt), 1)
    return True


import click
@click.command()
@click.option('-P','--can-port', default='vcan0', help='CAN Port')
@click.option('-i','--can-ids', default=range(41,47), multiple=True, type=int, help='CAN IDs')
def MainApp(can_port, can_ids):
  params = []
  for i in can_ids:
    # Each PyFakeVESC is its own client and server,
    vcan0 = {'type': 'can', 'driver': 'socketcan', 'port': can_port,  'bitrate': 500000, 'can_id': i}
    params.append( {'comm': vcan0, 'can_id': i,  'has_sensor': False, 'start_heartbeat': False, 'class':PyFakeVESC})
  mvesc = pyvesc.MultiVESC(params)

  vescs = []
  for i in can_ids:
    vescs.append(mvesc(i))

  stop_event = asyncio.Event()

  async def run_meas():
    while not stop_event.is_set():
      for v in vescs:
        LP_ALPHA = 0.05
        v.pid_pos += LP_ALPHA*(v.pid_ref - v.pid_pos) + random.uniform(-0.01,0.01)
        v.stream_status()
      await asyncio.sleep(0.01)

  async def shell():
    from traitlets.config.loader import Config
    from IPython.terminal.embed import InteractiveShellEmbed
    ipcfg = Config()
    ipcfg.InteractiveShell.confirm_exit = False
    ipcfg.using = 'asyncio'
    ipshell = InteractiveShellEmbed(
      config=ipcfg,
      banner1='>> Explorer Simulator interactive shell'
    )

    vesc = mvesc
    for c in vesc.controllers:
      v = c.get_firmware_version()
      print("[0x{:02X}] FW version: {:6s} - HW: {:15s} - UUID: 0x{:24X}".format(c.can_id if c.can_id is not None else 0, str(v),str(v.hw_name),v.uuid))

    ipshell()
    stop_event.set()

  asyncio.run((await asyncio.gather( shell(), run_meas()) for _ in '_').__anext__())

if __name__ == "__main__":
  MainApp()

# -*- coding: utf-8 -*-
"""
Created on Mon Oct 10 20:21:37 2022

@author: Jelle
"""
import PySimpleGUI as sg
import time

import struct
import socket

from collections import deque

sock_addr = '192.168.0.111'
sock_port = 4210

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.settimeout(1.0)

try:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.sendto(bytearray([0, 0, 0, 0]), ('<broadcast>', 4210))
    retval = []
    while 1:
            data, addr = s.recvfrom(1024)
            retval.append((data, addr))
except socket.timeout:
    pass
finally:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 0)
    sock_addr = addr[0]
for r in retval:
    data = struct.unpack("BBBBifff", r[0])


s.sendto(bytearray([1, 0, 0, 0]), (sock_addr, sock_port))
acc = [sg.Text('', key=f"__imu{i}__", size=(8,1)) for i in range(3)]
gyro = [sg.Text('', key=f"__imu{i}__", size=(8,1)) for i in range(3,6)]
mag = [sg.Text('', key=f"__mag{i}__", size=(8,1)) for i in range(3)]
adc = [sg.Text('', key=f"__adc{i}__", size=(8,1)) for i in range(4)]
layout = [acc, gyro, mag, adc, [sg.Quit()]]
window = sg.Window("streaming", layout=layout)
exceptOut = None

log = deque(maxlen=10)
t0 = time.time()
while 1:
    event, values = window.Read(timeout=0.01)
    try:
        res, addr = s.recvfrom(256)
        log.append({"head": struct.unpack("BBBB", res[0:4]),
                    "timestamp": time.time() - t0})
        type_code = res[1]
        if type_code == 2:
            text = [str(h) for h in struct.unpack('6h', res[4:])]
            for i, t in enumerate(text):
                window[f"__imu{i}__"].update(value=t)
        elif type_code == 3:
            text = [str(h) for h in struct.unpack('3h', res[4:])]
            for i, t in enumerate(text):
                window[f"__mag{i}__"].update(value=t)
            pass
        elif type_code == 4:
            text = [str(h) for h in struct.unpack('4h', res[4:])]
            for i, t in enumerate(text):
                window[f"__adc{i}__"].update(value=t)
            pass
    except socket.timeout:
        pass
    except Exception as e: 
        exceptOut = str(e)
        break
    if event in ('Quit', None):
        break
window.close()
s.sendto(bytearray([1, 0, 0, 0]), (sock_addr, sock_port))
while 1:
    try:
        res, addr = s.recvfrom(128)
        time.sleep(0.010)
    except socket.timeout:
        break
if exceptOut: sg.popup(exceptOut)
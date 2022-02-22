# -*- coding: utf-8 -*-
"""
Created on Sun Jun  7 19:57:03 2020

@author: Jelle
"""

import base_gui

import serial
import time
import struct
import math

REG_LED     = 22
REG_NAME    = 24
REG_SIZE    = 48

s = serial.Serial('COM5', baudrate = 115200, timeout = 2.0)
t_init = time.time()

def update():
    elapsed = time.time()-t_init
    if elapsed < 2:
        time.sleep(elapsed)
    
    s.flush()
    req = [1, 0, REG_SIZE, 0]
    data=['']*REG_SIZE
    s.write(req)
    res = s.read(size=4*REG_SIZE)

    if len(res) == 4*REG_SIZE:
        for i in range(0,9):
            data[i] = base_gui.readblock_float(res, i)
        for i in range(9,18):
            data[i] = base_gui.readblock_int16(res, i)
        data[22] = base_gui.readblock_uint8(res, 22)
        data[23] = base_gui.readblock_float(res, 23)
        for i in range(24,32):
            data[i] = base_gui.readblock_char(res, i)
        for i in range(32,36):
            data[i] = base_gui.readblock_int16(res, i)
        for i in range(36,38):
            data[i] = base_gui.readblock_float(res, i)
        for i in range(38,REG_SIZE):
            data[i] = base_gui.readblock_char(res, i)
    else:
        print("error: wrong number of reply bytes", len(res))

    return data

def set_name(name):
    name_size = 8
    flen = math.ceil(len(name) / 4)
    pad_bytes = '\0'*(4*name_size - len(name))
    if flen < name_size:
        s.write(
            struct.pack('4b', 2, REG_NAME, 8, 0) + name.encode() + pad_bytes.encode())
        
def set_color(rgb, tout):
    msg = [2, REG_LED, 1, 0,
           rgb[0], rgb[1], rgb[2], tout,
           0]
    print("led color:", msg)
    s.write(bytearray(msg))
    
def zero():
    magbias = [33, 135, -189]
    gyrobias = [-131, 28, -17]
    s.write(bytearray([2,32,4,0]) + struct.pack("8h", *magbias, 0, *gyrobias, 0))
    

def commit():
    s.write(bytearray([5,0,0,0]))
    
def empty():
    pass


import PySimpleGUI as sg
import numpy as np

def set_beta():
    layout = [[sg.Text("Beta: ", size=(16,1)), sg.Slider(range=(0,180), default_value=40, key='beta')],
              [sg.OK()]]
    window = sg.Window('Filtering', layout=layout)
    try:
        while 1:
            event, values = window.read()
            if event == None:
                break
            if event == 'OK':
                beta_degrees = values['beta']
                beta = np.sqrt(3/4) * np.pi * (beta_degrees/180)
                print("Setting beta:", beta)
                message = bytearray([2, 36, 2, 0]) + struct.pack('ff', beta, 0)
                s.write(message)
                
    finally:
        window.close()
        
##############################################################################

if __name__ == '__main__':
    try:
        gui = base_gui.BaseGUI(update,
                               commit,
                               zero,
                               set_color, 
                               set_name, 
                               REG_SIZE,
                               set_beta=set_beta)
        gui.main()
    finally:
        s.close()
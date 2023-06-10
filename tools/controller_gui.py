# -*- coding: utf-8 -*-
import time
import struct
import math
import socket
import threading
###############################################################################
REG_JOY     = 10
REG_LED     = 22
REG_NAME    = 24
REG_ADC     = 40
REG_SSID    = 48
REG_PASS    = 56
REG_SIZE    = 64

sock_addr = '192.168.0.165'
sock_port = 4210

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.settimeout(1.0)

def stream():
    req = [4, 0, 0, 0]
    s.sendto(bytearray(req), (sock_addr, sock_port))
    # t = threading.Thread(target=stream_recive)
    # t.start()
    acc = [sg.Text('', key=f"__text{i}__", size=(8,1)) for i in range(3)]
    gyro = [sg.Text('', key=f"__text{i}__", size=(8,1)) for i in range(3,6)]
    mag = [sg.Text('', key=f"__text{i}__", size=(8,1)) for i in range(6,9)]
    adc = [sg.Text('', key=f"__text{i}__", size=(8,1)) for i in range(9,13)]
    layout = [acc, gyro, mag, adc, [sg.Quit()]]
    window = sg.Window("streaming", layout=layout)
    tlast = time.time()
    exceptOut = None
    while 1:
        event, values = window.Read(timeout=0.01)
        try:
            res, addr = s.recvfrom(256)
            type_code = res[0]
            if type_code == 49:
                for i in range(0, 6):
                    text = base_gui.readblock_float(res[1:], i)
                    window[f"__text{i}__"].update(value=text)
            elif type_code == 50:
                for i in range(0, 3):
                    text = base_gui.readblock_float(res[1:], i)
                    window[f"__text{i+6}__"].update(value=text)
            elif type_code == 51:
                text = struct.unpack('4h', res[1:])
                for i in range(0,4):
                    window[f"__text{i+9}__"].update(value=text[i])
            elapsed = time.time() - tlast;
            tlast = time.time()
        except socket.timeout:
            pass
        except Exception as e: 
            exceptOut = str(e)
            break
        if event in ('Quit', None):
            break
    window.close()
    s.sendto(bytearray(req), (sock_addr, sock_port))
    while 1:
        try:
            res, addr = s.recvfrom(128)
            time.sleep(0.010)
        except socket.timeout:
            break
    if exceptOut: sg.popup(exceptOut)
    
def update():
    req = [1, 0, REG_SIZE, 0]
    data=['']*REG_SIZE
    try:
        s.sendto(bytearray(req), (sock_addr, sock_port))
        res, addr = s.recvfrom(1024)
        for i in range(0,9):
            data[i] = base_gui.readblock_float(res, i)
        for i in range(9,18):
            data[i] = base_gui.readblock_int16(res, i)
        data[22] = base_gui.readblock_uint8(res, 22)
        for i in range(24,32):
            data[i] = base_gui.readblock_char(res, i)
        for i in range(32,36):
            data[i] = base_gui.readblock_int16(res, i)
        for i in range(36,38):
            data[i] = base_gui.readblock_float(res, i)
        for i in range(40,48):
            data[i] = base_gui.readblock_int16(res, i)
        for i in range(48,REG_SIZE):
            data[i] = base_gui.readblock_char(res, i)
    except Exception as e:
            print(e)
    finally:
        return data

def set_name(name, name_size=8):
    flen = math.ceil(len(name) / 4)
    pad_bytes = '\0'*(4*name_size - len(name))
    if flen < name_size:
        s.sendto(
            struct.pack('4b', 2, REG_NAME, name_size, 0) + name.encode() + pad_bytes.encode(),
            (sock_addr, sock_port))
        
def set_color(rgb, tout):
    msg = [2, REG_LED, 2, 0,
           rgb[0], rgb[1], rgb[2], tout]
    s.sendto(
        bytearray(msg),
        (sock_addr, sock_port))
    
def commit():
    s.sendto(
        bytearray([5,0,0,0]), 
        (sock_addr, sock_port))

def read_adc():
    s.sendto(bytearray([1, REG_JOY, 2, 0]), (sock_addr, sock_port))
    res, addr = s.recvfrom(4*2)
    return struct.unpack('hhhh', res[0:8])

##############################################################################
import PySimpleGUI as sg

def search():
    id_req = [1, REG_NAME, 8, 0]
    s.sendto(bytearray(id_req), ('<broadcast>', 4210))
    return [s.recvfrom(4*8)]
        
def find_and_select():
    global sock_addr
    
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        controllers = {}
        retval = search()
        for name, addr in retval:
            try:
                strname = name.decode()
            except:
                strname = 'invalid name'
            controllers[strname] = addr[0]
        if not controllers:
            layout= [[sg.Text("No Controllers Found!")],
                     [sg.OK()]]
        else:
            layout = [[sg.Button(f"{k}") for k in controllers]]
        window = sg.Window("Select", layout=layout)
        event, values = window.read()
        if event in controllers.keys():
            sock_addr = controllers[event]
            print("controller IP is:", sock_addr)
        window.close()
    finally:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 0)

def zero_imu():
    magbias = [40, 58, -72]
    gyrobias = [6, 89, 98]
    s.sendto(bytearray([2,32,4,0]) + struct.pack("8h", *magbias, 0, *gyrobias, 0),
             (sock_addr, sock_port))
    yaw_offset = 0.0
    s.sendto(bytearray([2,23,1,0]) + struct.pack("f", yaw_offset),
             (sock_addr, sock_port))

def zero_adc():
    gsize = (16,1)
    joycol = [[sg.Text("Joystick Calib")]]
    for name in ("Up", "Down", "Left", "Right", "Mid"):
        joycol.append([sg.Button(name, size=gsize)])
    btncol = [[sg.Text("Button Calib")]]
    for name in ("A", "B", "Grip", "Stick"):
        btncol.append([sg.Button(name, size=gsize)])
    trigcol = [[sg.Text("Trigger Calib")]]
    for name in ("In", "Out"):
        trigcol.append([sg.Button(name, size=gsize)])
    layout = [[sg.Column(joycol), sg.Column(btncol), sg.Column(trigcol)],
              [sg.Spin([50,100,150,200,250,300], key="IN_SPREAD", size=gsize), sg.OK()],
              [sg.Text('', key=f'adc{i}', size=(20,1)) for i in range(4)]]
    window = sg.Window('Zeroing', layout=layout)
    try:
        reg_len = 8
        s.sendto(bytearray([1, REG_ADC, reg_len, 0]), (sock_addr, sock_port))
        res, addr = s.recvfrom(4*reg_len)
        calib = [h for h in struct.unpack('16h', res)]
        while 1:
            event, values = window.read(timeout=100)
            if event == None:
                break
            try:
                jx, jy, trig, button = read_adc()
                window['adc0'].update(value=str(jx))
                window['adc1'].update(value=str(jy))
                window['adc2'].update(value=str(trig))
                window['adc3'].update(value=str(button))
            except socket.timeout:
                pass
            
            if event == 'Left':
                calib[0] = jx
            if event == 'Right':
                calib[2] = jx
            if event == 'Down':
                calib[1] = jy
            if event == 'Up':
                calib[3] = jy
                
            if event == 'Mid':
                calib[4] = jx
                calib[5] = jy

            if event == 'Out':
                calib[6] = trig
            if event == 'In':
                calib[7] = trig
                
            if event == 'A':
                calib[8] = button - values['IN_SPREAD']
                calib[9] = button + values['IN_SPREAD']
            if event == 'B':
                calib[10] = button - values['IN_SPREAD']
                calib[11] = button + values['IN_SPREAD']
            if event == 'Grip':
                calib[12] = button - values['IN_SPREAD']
                calib[13] = button + values['IN_SPREAD']
            if event == 'Stick':
                calib[14] = button - values['IN_SPREAD']
                calib[15] = button + values['IN_SPREAD']
                
            if event == 'OK':
                s.sendto(bytearray([2, REG_ADC, 8, 0]) + struct.pack('16h', *calib),
                         (sock_addr, sock_port))
                break

    finally:
        window.close()
        
def setup_wifi():
    layout = [[sg.Text("SSID: ", size=(16,1)), sg.Input(key='ssid')],
              [sg.Text("Password: ", size=(16,1)), sg.Input(key='pass')],
              [sg.OK()]]
    window = sg.Window('Zeroing', layout=layout)
    try:
        while 1:
            event, values = window.read()
            if event == None:
                break
            if event == 'OK':
                ssid = values['ssid']
                ssid += '\0'*(4*8-len(ssid))
                pw = values['pass']
                pw += '\0'*(4*8-len(pw))
                message = bytearray([2, REG_SSID, 16, 0]) + ssid.encode() + pw.encode()
                s.sendto(message, (sock_addr, sock_port))
                
    finally:
        window.close()
        
def wipe_device():
    s.sendto(bytearray([2, 0, 64, 0]) + struct.pack('128h', *[0]*128),
         (sock_addr, sock_port))
##############################################################################
if __name__ == '__main__':
    try:
        import base_gui
        gui = base_gui.BaseGUI(update,
                               commit,
                               zero_imu,
                               set_color, 
                               set_name, 
                               REG_SIZE,
                               find=find_and_select,
                               adc=zero_adc,
                               WiFi=setup_wifi,
                               Wipe=wipe_device,
                               stream=stream)
        gui.col_len=32
        gui.main()
    finally:
        s.close()
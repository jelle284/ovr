# -*- coding: utf-8 -*-
"""
Created on Mon Jun  8 19:19:37 2020

@author: Jelle
"""

import struct
import PySimpleGUI as sg

def readblock_float(data, addr):
    f, = struct.unpack('f', data[4*addr:4*addr+4])
    return f"{f:.2f}"

def readblock_int16(data, addr):
    return ' '.join(
        [str(h) for h in struct.unpack('hh', data[4*addr:4*addr+4])] )

def readblock_char(data, addr):
        letters = []
        for c in struct.unpack('cccc', data[4*addr:4*addr+4]):
            try:
                letters.append(c.decode())
            except UnicodeError:
                letters.append('-')
        return ' '.join(letters)
    
def readblock_uint8(data, addr):
    return ' '.join(
            [str(b) for b in struct.unpack('BBBB', data[4*addr:4*addr+4])] )
    
class BaseGUI:
    def __init__(self, update, commit, zero, set_color, set_name, reg_size, **kwargs):
        self.update=update
        self.commit=commit
        self.zero=zero
        self.set_color=set_color
        self.set_name=set_name
        self.reg_size=reg_size
        self.user_btn = kwargs
        
        self.col_len = 16
    def led_dialog(self):
        try:
            layout = [[sg.Text(c, size=(3,1)) for c in 'RGB'],
                      [sg.Input('', size=(3,1), key=f'__{c}__') for c in 'RGB'],
                      [sg.Text('Timeout: '), sg.Input('', size=(3,1), key='__T__')],
                      [sg.OK(key='__OK__')],]
            window = sg.Window('led', layout=layout)
            event, values = window.read()
            if event == '__OK__':
                rgb = [int(values[f"__{c}__"]) for c in 'RGB']
                tout = int(values["__T__"])
                self.set_color(rgb, tout)
        finally:    
            window.close()
            
    def name_dialog(self):
        try:
            layout = [[sg.Multiline('', size=(16,4), key=f'__ID__')],
                      [sg.OK(key='__OK__')],]
            window = sg.Window('Enter a name', layout=layout)
            event, values = window.read()
            if event == '__OK__':
                name = values['__ID__']
                self.set_name(name[:-1])
        finally:    
            window.close()
    
    def main(self):
        cols = []
        for i in range(self.reg_size):
            row = [sg.Text(f"Reg {i}", size=(8,1)),
                   sg.Text('', size=(32,1),key=f"__REG{i}__")]
            if i%self.col_len == 0:
                cols.append([])
            cols[-1].append(row)
        
        
        userbtn = [sg.Button(btext, key=f'__{btext}__') for btext in self.user_btn]
        
        layout = [[sg.Button('Read', key='__READ__'),
                   sg.Button('Commit', key='__COMMIT__'),
                   sg.Button('Zero', key='__ZERO__'),
                   sg.Button('LED', key='__LED__'),
                   sg.Button('Identifier', key='__ID__'),],]
        for btn in userbtn:
            layout[0].append(btn)
        layout += [[sg.Column(c) for c in cols]]
        
        window = sg.Window('vr device config', layout=layout)
        window.finalize()
        
        try:
            while 1:
                event, values = window.read()
                
                if event == None:
                    break
                
                elif event == '__READ__':
                    data = self.update()
                    for i, d in enumerate(data):
                        window[f"__REG{i}__"].update(value=d)
                elif event == '__LED__':
                    self.led_dialog()
                elif event == '__ID__':
                    self.name_dialog()
                elif event == '__ZERO__':
                    self.zero()
                elif event == '__COMMIT__':
                    self.commit()
                elif event in (f'__{btext}__' for btext in self.user_btn):
                    self.user_btn[event.strip('__')]()
                
                # print(event, values, self.user_btn)
        finally:
            window.close()
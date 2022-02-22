# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 01:23:16 2020

@author: Jelle
"""
import PySimpleGUI as sg
from image_generate import draw_grid, get_blank
from matplotlib import pyplot as plt

################################ LAYOUT ######################################
X,Y,W,H = -1920, 0, 1920, 1080

################################# PLOT #######################################
vcenter = int(W/2)

fig, ax = plt.subplots()

plt.axis('off')

plt.rcParams['toolbar'] = 'None' # Remove tool bar (upper)
fig.canvas.window().statusBar().setVisible(False) # Remove status bar (bottom)
fig.tight_layout(pad=0)

manager = plt.get_current_fig_manager()
manager.window.setGeometry(X,Y,W,H)

im = get_blank(H,W)
imax = ax.imshow(im, interpolation='nearest', aspect='equal')

################################## GUI #######################################

layout = [[sg.Text(f'K{num}', key=f'T{num}'),
           sg.Slider(range=(-1,1),
                     default_value=0,
                     orientation='horizontal',
                     resolution = 0.005,
                     key=f'S{num}',
                     )] for num in (1,2)]

offset_frac = 10
layout.append([sg.Text(f'Offset', key=f'Toffset'),
               sg.Slider(range=(-vcenter/offset_frac,vcenter/offset_frac),
                         default_value=0,
                         orientation='horizontal',
                         resolution = 1,
                         key=f'Soffset',
                         )])

layout.append([sg.OK()])

window = sg.Window('distortion coefficients', layout)
try:
    while 1:
        event, values = window.Read()
        if not event: break
        if event == 'OK':
            k1 = values['S1']
            k2 = values['S2']
            offset = int(values['Soffset'])
            
            im.fill(255)
            
            if offset > 0:
                leye = im[:, offset:vcenter+offset, :]
                reye = im[:, vcenter-offset:W-offset, :]
            else:
                leye = im[:, :vcenter+offset, :]
                reye = im[:, vcenter-offset:, :]
                
            draw_grid(leye, k1, k2)
            draw_grid(reye, k1, k2)
            
            imax.set_data(im)
            fig.canvas.draw()

finally:
    window.close()
    plt.close('all')
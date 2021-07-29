from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from pyqtgraph.ptime import time
import serial
import sys
from prometheus_client import start_http_server, Gauge, Enum

port = sys.argv[1] #'/dev/ttyUSB0'
port_speed = 19200

g = Gauge('stream_data_value', 'Serlial monitor data', ["name"])

e = Enum('my_task_state', 'Description of enum',
        states=['starting', 'running', 'stopped'])
e.state('running')

app = QtGui.QApplication([])

num_samples = 500

pg.setConfigOptions(antialias=True)
p = pg.plot()
p.setWindowTitle('Serial plotter on '+port+' at '+str(port_speed))
p.setYRange(0, 38)
p.setXRange(0, num_samples)
p.resize(900,900)

labels = ["v_in", "v_out", "i_out", "duty", "v_max"]
#, "pwr"]

colors = {
	"v_in": 'r',
	"v_out": 'y',
	"i_out": 'm',
	"duty": 'g',
    "pwr": 'w',
    "v_max": "w"
}

curves = {}

data = {}

for l in labels:
    curve = pg.PlotCurveItem(pen=({'color': colors[l], 'width': 2}), skipFiniteCheck=True, name=l)
    p.addItem(curve)
    curve.setPos(0,0)
    curves[l] = curve
    data[l] = [0]

inf1 = pg.InfiniteLine(movable=True, angle=0, pen='y', bounds = [-2, 38], hoverPen=(0,200,0), label='Trickle out: {value:0.2f}V',
                       labelOpts={'color': 'y', 'movable': True, 'fill': (0, 0, 200, 100)})
inf1.setPos([0,13.8])

vout_high = pg.InfiniteLine(movable=True, angle=0, pen='y', bounds = [-2, 38], hoverPen=(0,200,0), label='Absorbtion/Bulk out: {value:0.2f}V',
                       labelOpts={'color': 'y', 'movable': True, 'fill': (0, 0, 200, 100)})
vout_high.setPos([0,14.7])

iout_max = pg.InfiniteLine(movable=True, angle=0, pen='r', bounds = [-2, 38], hoverPen=(0,200,0), label='Current limit: {value:0.2f}A',
                       labelOpts={'color': 'r', 'movable': True, 'fill': (0, 0, 200, 100)})
iout_max.setPos([0,2.5])

p.addItem(inf1)
p.addItem(vout_high)
p.addItem(iout_max)

raw=None

def parse(line):
    values = {}
    tag_parse = True
    tag = bytearray()
    val = bytearray()

    for c in line:
        if c == 10 or c == 13:
            break
        if tag_parse:
            if c==58:
                tag_parse=False
            else:
                tag.append(c)

        else:
            if c==32:
                values[tag.decode()] = float(str(val.decode()))
                tag_parse = True
                tag.clear()
                val.clear()
            else:
                val.append(c)

    values[tag.decode()] = float(str(val.decode()))
    return values

def update():
    global raw, port, port_speed, iout_max
    if not raw:
        raw = serial.Serial(port, port_speed)
    line = raw.readline()
    if len(line)>0:
        sys.stdout.write(line.decode("ascii"))
        if line[0]!=35:
            try:
                values = parse(line)
                imax = values["i_max"]
                iout_max.setPos([0,imax])
                for l in labels:
                    c = curves.get(l)
                    d = data.get(l)
                    v = values.get(l)
                    d.append(v)
                    g.labels(l).set(v)
                    while len(d)>num_samples:
                        d.pop(0)
                    c.setData(np.array(d, copy=False, dtype='float64'))
            except ValueError:
                print("value error - line skipped:", line)

    app.processEvents()

# app.show()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(5)

if __name__ == '__main__':
    # start_http_server(8000)
    app.exec()

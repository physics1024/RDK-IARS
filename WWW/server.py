#!/usr/bin/env python3
#import websockets
#import subprocess
import os
import threading
from time import sleep
from roslibpy import Topic, Ros
from flask import Flask, render_template, request, jsonify

alert = False
alert_lock = threading.Lock()

host = '127.0.0.1'
port = 9090

linear = 0.4
angular = 1

def falldownCall(msg):
    global alert
    with alert_lock:
        alert = (msg['data'] == 1)
    return

key_mapping = {
            'up'         : {'linear': {'x': linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': 0.0          }},
            'down'       : {'linear': {'x':-linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': 0.0          }},
            'left'       : {'linear': {'x': 0.0,        'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': angular }},
            'right'      : {'linear': {'x': 0.0,        'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z':-angular }},
            'up_left'    : {'linear': {'x': linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': angular }},
            'up_right'   : {'linear': {'x': linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z':-angular }},
            'down_left'  : {'linear': {'x':-linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': angular }},
            'down_right' : {'linear': {'x':-linear,'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z':-angular }},
            'stop'       : {'linear': {'x': 0.0,        'y': 0.0,    'z': 0.0           },
                           'angular': {'x': 0.0,        'y': 0.0,    'z': 0.0          }}
        }

ros = Ros(host=host, port=port)
ros.run()

car = Topic(ros, '/cmd_vel', 'geometry_msgs/Twist')
servo = Topic(ros, '/cmd_servo', 'std_msgs/msg/Int32')
falldown = Topic(ros, '/falldown_alert', 'std_msgs/msg/Int32')
fallDownSignal = falldown.subscribe(falldownCall)

def ahead():
    car.publish(key_mapping['up'])
    return
def back():
    car.publish(key_mapping['down'])
    return
def turnLeft():
    car.publish(key_mapping['up_left'])
    return
def turnRight():
    car.publish(key_mapping['up_right'])
    return
def backLeft():
    car.publish(key_mapping['down_left'])
    return
def backRight():
    car.publish(key_mapping['down_right'])
    return
def stop():
    car.publish(key_mapping['stop'])
    return
def camUp():
    servo.publish({'data': 4})
    return
def camDown():
    servo.publish({'data': 3})
    return
def camLeft():
    servo.publish({'data': 2})
    return
def camRight():
    servo.publish({'data': 1})
    return
def camReturn():
    servo.publish({'data': 5})
    return
def videoCapture():
    os.system('sudo systemctl start captureVideo')
    return
def videoDown():
    os.system('sudo systemctl stop captureVideo')
    return
    

app = Flask(__name__)

@app.route("/")
def index():
    if alert == False:
        return render_template("index.html")
    else:
        return render_template("alert.html")

@app.route("/ctrl")
def ctrl():
    if alert == False:
        return render_template("ctrl.html")
    else:
        return render_template("alert.html")

@app.route("/nav")
def nav():
    if alert == False:
        return render_template("nav.html")
    else:
        return render_template("alert.html")
    
@app.route("/mediplan")
def mediplan():
    if alert == False:
        return render_template("mediplan.html")
    else:
        return render_template("alert.html")

@app.route("/control", methods=['POST'])
def process():
    action = request.json.get('function')
    match action:
        case 'ahead':
            ahead()
        case 'back':
            back()
        case 'turnLeft':
            turnLeft()
        case 'turnRight':
            turnRight()
        case 'backLeft':
            backLeft
        case 'backRight':
            backRight
        case 'camUp':
            camUp()
        case 'camDown':
            camDown()
        case 'camLeft':
            camLeft()
        case 'camRight':
            camRight()
        case 'camReturn':
            camReturn()
        case 'videoCapture':
            videoCapture()
        case 'videoDown':
            videoDown()
    sleep(0.5)
    stop()
    return jsonify({"status": "success", "result": action})
@app.route("/check_fall", methods=['GET'])
def check_fall():
    with alert_lock:
        return jsonify({"status": 'alert', "alert": alert})

# 添加重置跌倒状态的API
@app.route("/reset_alert", methods=['POST'])
def reset_alert():
    global alert
    with alert_lock:
        alert = False
    return jsonify({"status": "success", "alert": alert})

if __name__ == '__main__':
    app.run()
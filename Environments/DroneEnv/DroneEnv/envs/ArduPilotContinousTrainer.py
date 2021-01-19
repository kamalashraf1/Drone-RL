import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
import threading
import time
import socket
import json
import math
from subprocess import Popen,PIPE
import os

arducopter_binary = "/cygdrive/e/Lab/2001/BTP/Ardupilot/ardupilot/build/sitl/bin/arducopter.exe"
defaults_path = "/cygdrive/e/Lab/2001/BTP/Winter/airsim-quadX.parm"
arducopter_options = [arducopter_binary,"-S","--model","airsim-copter","--speedup","1","--defaults",defaults_path,"-IO"]
shell_process = ['sh' ,'-c',"/usr/bin/python /cygdrive/e/Lab/2001/BTP/Winter/start_ardupilot.py"]
mav_proxy_process = ["mavproxy","--map", "--console", "--master", "tcp:127.0.0.1:5760","--out" ,"127.0.0.1:14550", "--out" ,"127.0.0.1:14551"]

class MiddleWare():
    """
    Class to maintain all middle utilities for modification
    """
    def __init__(self,other_ip,lim):
        #-- socket for incoming snesor data from AirSim
        self.other_ip = other_ip
        self.lim = lim
        self.create_connections()
        
    def create_connections(self):
        self.sensor_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sensor_in_address = ('0.0.0.0',9004)
        self.sensor_in.bind(self.sensor_in_address)

        # #-- socket for incoming snrvos data from ArduPilot
        self.servos_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.servos_in_address = ('localhost',9002)
        self.servos_in.bind(self.servos_in_address)

        #-- socket for sending modified sensor data to ArduPilot
        self.sensor_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
        self.sensor_out_address = ('localhost',9003)

        # #-- socket for sending servos data to AirSim
        self.servos_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
        self.servos_out_address = (self.other_ip,9006)

        #-- for error situations
        self.last_sensor_out = None
        self.last_servos_out = None
    
    def reset_connections(self):
        self.sensor_in.close()
        self.sensor_out.close()

    #-- Receive senor data from AirSim
    def receive_sensor_data(self):
        not_received = True
        j = 0
        while(not_received):
            try:
                self.sensor_in.settimeout(2)
                j += 1
                data, address = self.sensor_in.recvfrom(4096)
                self.last_sensor_out = data
                not_received = False
            except KeyboardInterrupt:
                print("Scokert Keyborad interrupt")
                exit(0)
            except socket.timeout:
                print("Socket Error")
                if j > 4:
                  exit(0)
                self.servos_out.sendto(self.last_servos_out,self.servos_out_address)
        sensors = json.dumps(data.decode('utf-8'))
        return json.loads(sensors)
    
    #-- Send json sensor data to Ardupilot
    def send_sensor_data_ardupilot(self,sensors):
        sensors = str(json.dumps(sensors)) + "\n"
        message = bytes(sensors, 'utf-8')
        self.last_sensor_out = message
        self.sensor_out.sendto(message,self.sensor_out_address)
    
    def modify_sensor_data(self,sensors,modifiers,modified_data):
        ## Only one modification for now
        sensors['gps']['lon'] = modified_data['gps']['lon'] + np.clip(modifiers[0],-(self.lim),self.lim)
        return sensors
        # Use following for extending the amount of data
        """
        modifiers = modifiers[0]
        sensors['gps']['lat'] = modified_data['gps']['lat'] + np.clip(modifiers[0],-(self.lim),self.lim)
        sensors['gps']['lon'] = modified_data['gps']['lon'] - self.lim
        sensors['gps']['lon'] += (-0.00001)
        sensors['gps']['alt'] += np.clip(modifiers[2],-(self.lim),self.lim)
        return sensors
        """

    def relay_servos_messages(self):
        data, address = self.servos_in.recvfrom(4096)
        self.last_servos_out = data
        self.servos_out.sendto(data,self.servos_out_address)

    def send_old_servos_airsim(self):
      self.servos_out.sendto(self.last_servos_out,self.servos_out_address)

    def convert_to_numpy(self,sensors):
        sensor_arr = []
        sensor_arr = sensor_arr + [sensors['gps']['lat'], sensors['gps']['lon'], sensors['gps']['alt']]
        # sensor_arr = sensors['imu']['angular_velocity']
        # sensor_arr = sensor_arr + sensors['imu']['linear_acceleration']
        # sensor_arr = sensor_arr + sensors['velocity']['world_linear_velocity']
        # sensor_arr = sensor_arr + [sensors['pose']['roll'], sensors['pose']['pitch'], sensors['pose']['yaw']]
        return np.array(sensor_arr)

class ArduPilotContinuousTrainer(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self,other_ip,lim,dest,adverse,reward_fn=0):
    self.netHandler =  MiddleWare(other_ip,lim)
    self.state = np.zeros((3,))
    self.sensors = []
    self.home = LocationGlobalRelative(-35.363262, 149.165237, 40)
    self.adverse_location = adverse
    self.dest = dest
    self.start = True
    self.reward_fn = reward_fn
    self.logger = open('logging.txt','w')
    self.logger_2 = open('logging_2.txt','w')
    self.stop = True
    self.times = 0
    self.start = True
    self.command_in_address = ('0.0.0.0',9007)
    self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.command_socket.bind(self.command_in_address)
    self.command_out_address = (other_ip,9008)
    self.ardupilot = None

  def step(self, action):
    self.modified_data = self.netHandler.modify_sensor_data(self.sensors,action,self.modified_data)
    self.netHandler.send_sensor_data_ardupilot(self.modified_data)
    self.netHandler.relay_servos_messages()
    self.sensors = json.loads(self.netHandler.receive_sensor_data())
    self.logger_2.write('Changed: ' + str(self.modified_data['gps']) + '\n')
    self.logger_2.write('Original: ' + str(self.sensors['gps']) + '\n')
    self.logger.write(str(self.diff) + '\n')
    self.state = self.netHandler.convert_to_numpy(self.sensors)
    self.costs = self.calculate_costs()
    self.done = self.situation_done()
    self.times = self.times+1 
    normalized_array = 1000*np.array([self.state[0] - self.home.lat, self.state[1] - self.home.lon, self.state[2]])
    return normalized_array ,self.costs, self.done, {}
  
  def calculate_costs(self):
    longitude_diff = abs(self.adverse_location.lon - self.state[1])
    if self.reward_fn == 0:
      return -longitude_diff
    cost = self.diff - longitude_diff
    self.diff = longitude_diff
    if cost >= 0 :
      return 50000.0*cost
    return 100000.0*cost

  def reset(self):
    if not self.start:
        end_message = b"Kill AirSim"
        self.command_socket.sendto(end_message,self.command_out_address)
        self.command_socket.sendto(end_message,('localhost',9011))
        self.ardupilot.kill()
        self.mavproxy.kill()
        os.system("taskkill /im mavproxy.exe")
        self.vehicle.close()

    start_message = b"Start AirSim"
    self.command_socket.sendto(start_message,self.command_out_address)
    self.command_socket.settimeout(10)
    while True:
      try:
        data ,address = self.command_socket.recvfrom(4096)
        break
      except:
        print("Timeout Error in Reset")
        self.command_socket.sendto(start_message,self.command_out_address)
    self.start = False
    self.state = np.zeros((15,))
    self.times = 0
    time.sleep(2)

    # Starting up Ardupilot Binary
    self.ardupilot = Popen(shell_process)
    self.mavproxy = Popen(mav_proxy_process)
    ## Now starting the thread
    self.stop = False
    thd = threading.Thread(target = self.simple_loop)
    thd.start()
    time.sleep(2)
    self.connect_to_vehicle()
    self.vehicle.simple_goto(self.dest)
    self.stop = True
    self.start = False
    self.diff = abs(self.adverse_location.lon - self.state[1])
    self.netHandler.relay_servos_messages()
    normalized_array = 1000*np.array([self.state[0] - self.home.lat, self.state[1] - self.home.lon, self.state[2]])
    return normalized_array

  def render(self, mode='human', close=False):
    return
  
  def end_all(self):
    end_message = b"Kill AirSim"
    self.command_socket.sendto(end_message,self.command_out_address)
    self.command_socket.sendto(end_message,('localhost',9011))
    self.ardupilot.kill()
    os.system("taskkill /im mavproxy.exe")
    self.mavproxy.kill()
    self.vehicle.close()

  def simple_loop(self):
    try:
      while(not self.stop):
        for i in range(10):
          self.netHandler.relay_servos_messages()
          self.sensors = json.loads(self.netHandler.receive_sensor_data())
          self.modified_data = self.sensors
          self.state = self.netHandler.convert_to_numpy(self.sensors)
          self.netHandler.send_sensor_data_ardupilot(self.sensors)
    except KeyboardInterrupt:
      return
  
  def not_at_home(self):
    dest = self.get_distance_metres(self.home,self.state[0],self.state[1])
    if self.start :
      self.start = False
      return dest > 15
      return False
    else:
      return dest > 15
  
  def get_distance_metres(self,aLocation1, lat,lon):
    dlat = aLocation1.lat - lat
    dlong = aLocation1.lon - lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
  
  def situation_done(self):
    target_dest = self.get_distance_metres(self.dest,self.state[0],self.state[1])
    adver = self.get_distance_metres(self.adverse_location,self.state[0],self.state[1])
    lat_diff = abs(self.state[0] - self.dest.lat)
    lat_diff_adv = abs(self.modified_data['gps']['lat'] - self.dest.lat)

    if target_dest < 20 or adver < 20 or lat_diff < 0.00002 or lat_diff_adv < 0.00002:
      print("situation_done " + str(self.times) +" " + str(target_dest) + " " + str(adver))
      print("situation done " + str(lat_diff) + " " + str(lat_diff_adv))
      print("lat : " + str(self.state[0]) + " lon : " + str(self.state[1]))
    return target_dest < 20 or adver < 20 or lat_diff < 0.00002 or lat_diff_adv < 0.00002 or self.vehicle.location.global_relative_frame.alt < 20 

  def connect_to_vehicle(self):
    self.connection_string = "tcp:127.0.0.1:5760"
    ## Use this for connection over udp
    self.connection_string = "udp:127.0.0.1:14550"
    self.vehicle = connect(self.connection_string, wait_ready=False)
    self.vehicle.wait_ready(True, timeout=300)
    self.arm_and_takeoff()

  def arm_and_takeoff(self):
    print("Basic pre-arm checks")
    while not self.vehicle.is_armable:
        # print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    self.vehicle.mode = VehicleMode("GUIDED")
    self.vehicle.armed = True
    while not self.vehicle.armed:
        # print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    self.vehicle.simple_takeoff(40)  # Take off to target altitude
    while True:
        print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
        if self.vehicle.location.global_relative_frame.alt >= 40 * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
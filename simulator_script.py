import sys
import socket
import time
import json
from json import encoder
from subprocess import Popen,PIPE

if len(sys.argv) > 1:
    other_ip = sys.argv[1]
else:
    other_ip = 'localhost'

commands_scoket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
commands_in_address = ('localhost',9008)
commands_scoket.bind(commands_in_address)
commands_out_address = (other_ip,9007)

airsim_binary = "/cygdrive/e/Lab/2001/BTP/Downloads/Blocks/Blocks/Binaries/Win64/Blocks.exe"
aisrim_options = [airsim_binary,"-ResX=480","-RessY=320","-windowed"]

arducopter_binary = "/cygdrive/e/Lab/2001/BTP/Ardupilot/ardupilot/build/sitl/bin/arducopter.exe"
defaults_path = "/cygdrive/e/Lab/2001/BTP/Winter/airsim-quadX.parm"
arducopter_options = [arducopter_binary,"-S","--model","airsim-copter","--speedup","1","--defaults",defaults_path,"-IO"]

def process_maintainance():
    try:
        while True:
            # wait for receiving reset instruction from Training Process
            data , message = commands_scoket.recvfrom(4096)
            
            # start airSim simulator executable
            airsim = Popen(aisrim_options)

            # send signal post processes initiation (assuming they must have started by now)
            started_message = b"AirSim Started"
            commands_scoket.send(started_message,commands_out_address)

            # receive stop signal from the Training Process
            data , message = commands_scoket.recvfrom(4096)
            # kill both lauunched processes
            print("Killing AirSim")
            airsim.kill()

    except KeyboardInterrupt:
        print("Keyboard Interrupt Received")
        exit(0)

def main():
    process_maintainance()
main()
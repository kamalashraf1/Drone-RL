from subprocess import Popen,PIPE
import time
import socket

arducopter_binary = "/cygdrive/e/Lab/2001/BTP/Ardupilot/ardupilot/build/sitl/bin/arducopter.exe"
defaults_path = "/cygdrive/e/Lab/2001/BTP/Winter/airsim-quadX.parm"
defaults_path = "/cygdrive/e/Lab/2001/BTP/Ardupilot/ardupilot/Tools/autotest/default_params/copter.parm,default_params/airsim-quadX.parm"
defaults_path = "/cygdrive/e/Lab/2001/BTP/Ardupilot/ardupilot/Tools/autotest/default_params/copter.parm,/cygdrive/e/Lab/2001/BTP/Ardupilot/ardupilot/Tools/autotest/default_params/airsim-quadX.parm"
arducopter_options = [arducopter_binary,"-S","--model","airsim-copter","--speedup","1","--defaults",defaults_path,"-IO"]

mavProxy = ["/usr/bin/cygstart","-w","mavproxy.exe","--map","--console","--out","127.0.0.1:14550","--out","127.0.0.1:14551","--master","tcp:127.0.0.1:5760","--sitl","127.0.0.1:5501"]

mv = ["cmd.exe","D:\\Softwares\\MAVProxy\\mavproxy.exe" , "--map","--console","--out","127.0.0.1:14550","--out","127.0.0.1:14551","--master","tcp:127.0.0.1:5760","--sitl","127.0.0.1:5501"]
scr = "mavproxy.exe --map --console --out 127.0.0.1:14550 --out 127.0.0.1:14551 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501"
mv = ["cmd.exe /k D:\\Softwares\\MAVProxy\\mavproxy.exe"]
airsim_binary = "/cygdrive/e/Lab/2001/BTP/Downloads/Blocks/Blocks/Binaries/Win64/Blocks.exe"
aisrim_options = [airsim_binary,"-ResX=480","-RessY=320","-windowed"]

win_options = [r'D:\cygwin64\bin\run.exe','--wait','-p'] + [arducopter_binary] + arducopter_options
win_options = [r'D:\cygwin64\bin\run.exe','--wait','-p'] + [airsim_binary] + aisrim_options
win_opotions = [r'D:\cygwin64\bin\run.exe','--wait','-p']

## Netowrking for getting comamnd to stop

commands_scoket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
commands_in_address = ('localhost',9011)
commands_scoket.bind(commands_in_address)


ardupilot = Popen(arducopter_options)

try:
    a,b = commands_scoket.recvfrom(4096)
except:
    ardupilot.kill()
    print("Interrupt")
    exit(0)

ardupilot.kill()


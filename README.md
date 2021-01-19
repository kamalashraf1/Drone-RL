# Drone-RL

## Setup
### Fisrt set the gym environment by following steps
 
- `cd Environments\DroneEnv`
- `pip install -e .`

### For training:

First start the simulator script:

- `python simulator_script.py training_script_ip`

Then start trianing algorithm:

- `python gps_only.py simulator_ip`
- `python multiple_sensors.py simulator_ip`

### NOTE

The locations of some scripts are specified as per Windows+cygwin setup. Modify the paths as per you directory setup

- Gym environments use `start_ardupilot.py`
- Change the location of this script in the gym environments scoure file
- For `simulator_script.py` modify the paths to Airsim binary

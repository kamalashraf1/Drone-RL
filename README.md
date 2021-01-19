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


## Credits and References

- AirSim binaries can be downloaded from [here](https://microsoft.github.io/AirSim/use_precompiled/)
- Follow [this](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) link to setup ArduPilot
- Setup the SITL and then locate the `arducopter.exe` binary that is needed by the gym environments
- The Deep Deteminsitic Policy Gradient (DDPG) implementation is based on [Tensorflow and Keras](https://keras.io/examples/rl/ddpg_pendulum/)
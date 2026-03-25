# whiteboat-tools

## Dependencies
```
pip install pyserial
```

## Cloning...
### [Creating a Token](https://github.com/settings/tokens)
#### Generate new token > Generate new token (classic)
```
git clone -b develop https://<your_user>:<your_token>@github.com/hydrone-furg/whiteboat-tools
```

## To use SITL:
### [Rover SITL](https://ardupilot.org/dev/docs/rover-sitlmavproxy-tutorial.html)
```
git clone --recurse-submodules git@github.com:ArduPilot/ardupilot.git
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
cd ~/ardupilot/Rover
sim_vehicle.py --map --console
```

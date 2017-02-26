# Additional scripts of SkyScanner main integration

## FlightGear simulation

`fly_malolo1.sh` starts FlightGear with the malolo1 UAV, defined in `FlightGear/Aircraft/Malolo1`.

Some steps are necessary to connect the skyscanner ROS package with FlightGear:

 * Copy the contents of `FlightGear/Protocol` to the Protocol folder in FlightGear root (In ubuntu 14.04 it is in `/usr/share/games/FlightGear`) These files contain the communication protocol definition.
 * The port numbers are defined in `fly_malolo1.sh` and should match with the defined ones in skyscanner's conf files.

The easystar UAV is availabe too but beware it is unstable with the default parameters of FlightGear's autopilot.

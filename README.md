# MiAM_eurobot2019
## Electronics and code of MiAM's robot for Eurobot 2019.

This repo contains code and electronics design files related the 2019 Eurobot competition of MiAM,
a French team of former students from MINES ParisTech and Arts et Métiers schools.

For older files related to the 2018 robot setup, see [MiAM_eurobot2018](https://github.com/matthieuvigne/MiAM_eurobot2018) repo.
The 2019 repo started from the 2018, but has adapted to the new robot architecture (motors, sensors...)

See [MiAM website](https://www.miam-robotique.fr/) (in French).


### Contents
 - [**ArduinoFunnyAction**](./ArduinoFunnyAction) and [**ArduinoSlave**](./ArduinoSlave): arduino code for the funny action (experiment) and for the Raspberry pi slave
 of the main robot.
 - [**ConfigRPi**](./ConfigRPi): Inscruction for configuring a raspberry pi to have headless wifi connection and to run our code.
 - [**Electronics**](./Electronics): wiring diagram and PCBs for the main robot.
 - [**RobotCode**](./RobotCode): onboard C++ code, running on Beaglebone Black or Raspberry pi processor.
 - [**miam_py**](./miam_py): a python package for reading and plotting a log file from the robot.
 - [**StrategyViewer**](./StrategyViewer): A GTK+-based viewer for the trajectories for the robots.

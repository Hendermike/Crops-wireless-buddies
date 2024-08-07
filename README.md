# Crops wireless buddies

This is collection of firmwares for a series of modular hardware devices that allow the environment control of hydroponic system.
The devices are communicated and coordinated through a series of nRF24L01-based links.

### SolutionRF

Integrates DFRobot gravity pH and EC sensors with a nRF24L01 to provide current state of the liquid medium to the system.
Also implements functions for the calibration of both sensors through a dedicated procedure.

### Solution RF-UI

Implements minimalistic button-based HMI to enable "Normal operation" and "Calibration" mode switching of the Solution RF module. Eventually, this module might get merged with SolutionRF.

### DoserRF

Using Solution RF module's data, activates or deactivates a series of peristaltic pumps, controlled by a H-bridge DC motor controller, in order to keep EC (N+P+K+Cal+Mg Nutrients) and pH between desired levels. It also indenties when EC surpasses dangerous levels for the crops, requesting Pumper RF a liquid medium cleaning routine.

### PumperRF

Integrates a US sensor, and 3 AC water pumps to keep water level (WATER_IN, WATER_OUT and WATER_CYCLE) and circulation of the whole system. (TO-DO) Also, is responsible for the liquid medium cleaning routine, when requested by DoserRF.

## TO-DOs

### StreamerRF

Is intended to read all nRF relevant logical channels to keep track of

### SoilRF



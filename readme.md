# PZEM-004T Drivers

Esp-idf library for Peacefair **PZEM-004T-10A** and [**PZEM-004T-100A v3.0**](https://innovatorsguru.com/pzem-004t-v3/) Energy monitor using the serial interface.  The drivers are created as the component, so that easily incoporates into the esp-idf project. The poject is fully created in the c for better compatibility.

## Features

Measures Voltage, Current, Power, Energy, **Power Factor** and **Frequency**



This repo includes the data sheet of the modules in the doc folder.

This example is drived from the repo [https://github.com/olehs/PZEM004T](). Have a shout out to the developer.


## Shortcomings

This driver cannot set the address to the pzem module. We need to setup using the other program, example like the arduino library.

![N|Solid](https://javier.rodriguez.org.mx/itesm/2014/tecnologico-de-monterrey-black.png)
# Open Source Weather Station
## _Design and implementation of a sensing platform to assess and forecast environmental conditions._
>Designed by Ing. Antonio Carlos Rivera Corona
>Advisor Dr. Pedro Ponce Cruz
>Co-Advisor Dr. Arturo Molina Gutiérrez
>A01337294@tec.mx
> México, CDMX 
>2023

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

## Abstract
Local environments are altered due to natural phenomena caused by climate change. Some nature changes are, for instance, heat waves, floods, wildfires, and air pollution, among others. Monitoring these alterations is compulsory for users who require a better understanding of their surroundings, such as farmers, transportation companies, and climate researchers. There are already solutions implemented to address this, such as fixed weather stations, satellite images, or drone mapping; However, the limitation of these approaches is that they are unaffordable for small users due to high-cost sensors and the trained personnel needed for operation and maintenance. Also, they have limited coverage and do not share information with the public. Therefore, this paper shows the design, development, and evaluation of an open-source automatic weather station (OSWS) based on low-cost sensors that monitor environmental variables, including temperature, relative humidity, atmospheric pressure, CO2 concentration, and particulate matter (PM1, PM2.5, and PM10). This station provides a fast solution for in-site measurements for different users; it can monitor the variables remotely and forecast values within a short-time period based on the historical data captured using the ARIMA model.

> This repository contains the source code for the open source weather station (OSWS) device.

## Features
> This OSWS measures in real time:
- Air Temperature
- Relative Humidity
- Atmospheric Pressure
- CO2 concentration
- PM1
- PM2.5
- PM10
>These measurements are shown in an on-board 2.4" display

> The interval measurement time is configurable:
- 30 seconds
- 1 minute
- 10 minutes
- 30 minutes
- 1 hour
- 12 hours
- 24 hours
> The device stores the data with its timestamp on a SD card in a .CSV forman and on the ThingSpeak platform via WiFi

> This device can be powered by:
- Internal Li-Ion battery
- Solar Panel
- AC Plug

## Tech

The device includes:
- Two ARM M3 Microcontrollers (STM32F103C8T6).
- A WiFi Modem (ESP8266).
- A 2.4" On-board display
- Two designed PCBs.
- SD card module.
- Shielded wires to prevent external interferences.
- IP65 case to protect internal components from dust and water.
- Internal RTC for timestamp.
- PLA-Design accesories to cover the sensors from rain and direct sunlight. 


## Installation

The OSWS is plug-and-play. When powered, the station will start to measure automatically with a measurement interval of 30 seconds by default.

## License

MIT

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [dill]: <https://github.com/joemccann/dillinger>
   [git-repo-url]: <https://github.com/joemccann/dillinger.git>
   [john gruber]: <http://daringfireball.net>
   [df1]: <http://daringfireball.net/projects/markdown/>
   [markdown-it]: <https://github.com/markdown-it/markdown-it>
   [Ace Editor]: <http://ace.ajax.org>
   [node.js]: <http://nodejs.org>
   [Twitter Bootstrap]: <http://twitter.github.com/bootstrap/>
   [jQuery]: <http://jquery.com>
   [@tjholowaychuk]: <http://twitter.com/tjholowaychuk>
   [express]: <http://expressjs.com>
   [AngularJS]: <http://angularjs.org>
   [Gulp]: <http://gulpjs.com>

   [PlDb]: <https://github.com/joemccann/dillinger/tree/master/plugins/dropbox/README.md>
   [PlGh]: <https://github.com/joemccann/dillinger/tree/master/plugins/github/README.md>
   [PlGd]: <https://github.com/joemccann/dillinger/tree/master/plugins/googledrive/README.md>
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
   [PlMe]: <https://github.com/joemccann/dillinger/tree/master/plugins/medium/README.md>
   [PlGa]: <https://github.com/RahulHP/dillinger/blob/master/plugins/googleanalytics/README.md>

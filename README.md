# Hack The Planet Smart Bridge Firmware
C Firmware running on the Smart Bridge micro controller

### Prerequisites

What things you need to use this code-base:

* [LoRa Module made by Murata](https://www.murata.com/products/connectivitymodule/lpwa/lora) on some kind of a development or prototype board.
* [Visual Studio Code](https://code.visualstudio.com/)
* [Arduino plugin for VS Code](https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino)
* [St-link](https://www.st.com/en/development-tools/st-link-v2.html)
* [Arduino IDE](https://www.arduino.cc/en/Main/Software)
* [ArduinoCore for STM32L0](https://github.com/IRNAS/ArduinoCore-stm32l0)
* Installed Arduino libraries: LibLacuna-0.96, [Adafruit BME280 library](https://github.com/adafruit/Adafruit_BME280_Library), [Adafruit Sensor library](https://github.com/adafruit/Adafruit_Sensor)

### Installing

* Install Arduino IDE, make sure that you click on Windows installer, not on Windows app.
* Install ArduinoCore for STM32L0, follow installation rules [here](https://github.com/IRNAS/ArduinoCore-stm32l0)
* Install Visual Studio Code and Arduino plugin 
* clone our repository with `git clone https://github.com/IRNAS/irnas-lorawan-base` and open main directory in VS Code

## Secrets
Put the LoRa keys in `secrets.h` file

    #define RELAY_NETWORKKEY[] = {...};
    #define RELAY_APPKEY[] = {...};
    #define RELAY_DEVICEADDRESS[] = {...};


## Build and flash 

* Connect St-link to LoRa module and PC
* In VS Code, bottom right click select board and from select board dropdown menu select `IRNAS-env-module-L072Z` and close configuration tab
* Click Arduino: Upload button in top right, code will be compiled and flashed to the board.

## Using release_build.py script

'Release_build.py' will create a version.h file with provided tag, do a commmit and push and automatically create a build release on GitHub.

## Authors

* **Luka Mustafa** - *Initial work, ongoing work* - [SloMusti](https://github.com/SloMusti)
* **Marko Sagadin** - *documentation, ongoing work* - [SkobecSlo](https://github.com/SkobecSlo)
* **Thijs Suijten** - *work for hack the poacher setup* - [tsuijten](https://github.com/tsuijten)


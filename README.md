# Senior-Design---Stored-Battery-Monitering-System
* Install VSCode
* Install gitbash and setup your git account
* Install GNU ARM toolchain https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.07/gcc-arm-none-eabi-10.3-2021.07win32/gcc-arm-none-eabi-10.3-2021.07-win32.exe . Install this in C:/ program files (x86)/ GNU ARM Embedded Toolchain folder and add the bin file in there to the path
* Install Mingw and install it in C:/Mingw, add the bin and the msys folder to the environment variables path
* Install the C/C++ extension for VSCode
* Install the NRF Connect desktop app

## How to compile and flash code?
* navigate to the src/pca10056(for the dev kit) or the pca10059/armgcc
* Start a git bash session in this folder, and run the make command. That will compile the code for you.
* To delete the build folder, run make clean.
* To flash the code, open the NRFconnect app and open the programmer. Select the hex file from the _build folder. Also select the soft device hex file from the soft device folder
* Select the write command to write these commands to the board


To run the BLE signal strength tool
* Install Python 3.7.9[That specific version]
* Install the python library called Blatann


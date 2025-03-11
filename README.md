# LPS3x
MicroPython module for working with the LPS3x ambient air pressure sensor from STMicroelectronics.

# Motivation
Due to the dominance of air pressure sensors from the company starting with the letter "B" in my repositories,
I decided to "dilute" it (the dominance) with a sensor from another manufacturer.
In this case, it is STMicroelectronics.

# Connection
Simply connect the board with LPS3x to Arduino, ESP or any other board with MicroPython firmware.

# Power
The supply voltage of LPS3x is 1.7..3.6 Volts!
1. VCC
2. GND
3. SDA
4. SCL

# Firmware
Upload the MicroPython firmware to the NANO board, PICO RP2040 (ESP, etc.), and then the files: main.py, lps3xmod.py, and the entire sensor_pack_2 folder.
Then open main.py in your IDE and run it.

# Pictures

## Integrated Development Environment
![alt text](https://github.com/octaprog7/LPS3x/blob/master/pics/ide_0.png)
## Board
![alt text](https://github.com/octaprog7/LPS3x/blob/master/pics/board_0.png)


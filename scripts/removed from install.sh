# Installing Adafruit_Dependecies for PCA9685 Board
mkdir ~/Adafruit_Dependecies

# CircuitPython installation --> https://github.com/adafruit/circuitpython
cd ~/Adafruit_Dependecies
git clone https://github.com/adafruit/circuitpython.git
cd ~/Adafruit_Dependecies/circuitpython
sudo python setup.py install

# CircuitPython_BusDevice installation --> https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
cd ~/Adafruit_Dependecies
git clone https://github.com/adafruit/Adafruit_CircuitPython_BusDevice.git
cd ~/Adafruit_Dependecies/Adafruit_CircuitPython_BusDevice
sudo python setup.py install

# CircuitPython_Register installation --> https://github.com/adafruit/Adafruit_CircuitPython_Register
cd ~/Adafruit_Dependecies
git clone https://github.com/adafruit/Adafruit_CircuitPython_Register.git
cd ~/Adafruit_Dependecies/Adafruit_CircuitPython_Register
sudo python setup.py install

# CircuitPython_PCA9685 installation --> https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
cd ~/Adafruit_Dependecies
git clone https://github.com/adafruit/Adafruit_CircuitPython_PCA9685.git
cd ~/Adafruit_Dependecies/Adafruit_CircuitPython_PCA9685
sudo python setup.py install


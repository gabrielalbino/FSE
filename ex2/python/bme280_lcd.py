import smbus2
import bme280
import RPi_I2C_driver
from time import *

porta_i2c = 1
endereco = 0x76
bus = smbus2.SMBus(porta_i2c)

calibracao_parametros = bme280.load_calibration_params(bus, endereco)
mylcd = RPi_I2C_driver.lcd()


dado = bme280.sample(bus, endereco, calibracao_parametros)

while(True):
  stringNome = "Gabriel T:" + "{:.2f}".format(dado.temperature)
  stringDado = "P:" + "{:.2f}".format(dado.pressure) + " H:" + "{:.2f}".format(dado.humidity)
  print(stringNome)
  print(stringDado)
  mylcd.lcd_display_string(stringNome, 1)
  mylcd.lcd_display_string(stringDado, 2)
  sleep(1)
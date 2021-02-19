# AD5933 12-bit network analyzer 
# https://github.com/dxdevoe/pmodia
#
# Adapted with significant modification from 
# https://github.com/wheelin/Impedance_raspi_ad5933
#
#
# Notes:
#
# * PmodIA module must use 3.3V power or the status register
#   may not behave correctly.
#
# * SEL pin must be tied to ground
#
# * PModIA / AD5933 specs allows for 100 Ohm - 10MOhm impedance range,
#   but the system operates poorly near the ends of this range, and
#   must be calibrated for a given frequency range. In general, keep
#   the sweep range small, and calibrate with a known Z over this
#   range.
#
# * Be sure to use a suitable excitation voltage and receive stage
#   gain to prevent ADC saturation, which can result in strange output.

import smbus
from math import atan, sqrt
from time import sleep
import csv
from datetime import datetime

# Create log file name
def f_name():
  i = datetime.now()
  return "log-{:0>2}_{:0>2}_{}-{:0>2}_{:0>2}_{:0>2}.csv".format(
          i.day, i.month, i.year, i.hour, i.minute, i.second)

# Registers
CONTROL_REG0 = 			0x80
CONTROL_REG1 = 			0x81

FREQ_MIN_REG0 = 		0x82
FREQ_MIN_REG1 = 		0x83
FREQ_MIN_REG2 = 		0x84

FREQ_INC_REG0 = 		0x85
FREQ_INC_REG1 = 		0x86
FREQ_INC_REG2 = 		0x87

INC_NUM_REG0 = 			0x88
INC_NUM_REG1 = 			0x89

STTL_TIME_CY_NUM_REG0 =         0x8A
STTL_TIME_CY_NUM_REG1 =         0x8B

STATUS_REG = 			0x8F

TEMP_DATA_REG0 = 		0x92
TEMP_DATA_REG1 = 		0x93

REAL_DATA_REG0 = 		0x94
REAL_DATA_REG1 = 		0x95

IMG_DATA_REG0 = 		0x96
IMG_DATA_REG1 = 		0x97

MAX_FREQ = 			100e3
MIN_FREQ = 			1e3

# Control register commands (0x80)
INIT_WITH_START_FREQ = 	(0b0001 << 4)
START_FREQ_SWEEP = 	(0b0010 << 4)
INCREMENT_FREQ = 	(0b0011 << 4)
REPEAT_FREQ = 		(0b0100 << 4)
MEASURE_TEMP = 		(0b1001 << 4)
POWER_DOWN = 		(0b1010 << 4)
STANDBY = 		(0b1011 << 4)

# Control register commands (0x81)
RESET = 		0b00010000

# Status register state values
TEMP_MEAS_STATUS =      0b0001
IMP_MEAS_STATUS =       0b0010
IMP_SWEEP_STATUS =      0b0100

# Peak-to-peak excitation voltage code map
# codes 1,2,3,4 = 2V, 1V, 0.4V, 0.2V peak-peak
EX_VOLTAGE_CODES =      [0b00, 0b11, 0b10, 0b01]


class AD5933:
  def __init__(self, clk, port_number, device_address):
    self.bus = smbus.SMBus(port_number)  # port 0 for I2C0, 1 for I2C1
    self.address = device_address        # check using i2cdetect -y 0 (or 1)
    self.clk = clk                       
    self.set_clock()

  def init(self):
    self.set_ex_voltage(2)  # select excitation voltage
    self.set_PGA_gain(1)    # select gain factor
    self.set_settling_time(511)  # settling time (cycles)

  def make_imp_measure(self, fo, df, steps):
    freq = fo
    img = 0
    real = 0
    self.set_freq_range(fo, df, steps)
    self.send_cmd(STANDBY)
    self.send_cmd(INIT_WITH_START_FREQ)
    self.send_cmd(START_FREQ_SWEEP)
    with open(f_name(), 'wt') as csvfile:
      spamwriter = csv.writer(csvfile, delimiter=',')
      spamwriter.writerow(['Freq', 'Real', 'Img'])
      while not self.check_status(IMP_SWEEP_STATUS):
        while not self.check_status(IMP_MEAS_STATUS):
          pass
        img = self.read_img_reg()
        real = self.read_real_reg()
        spamwriter.writerow([str(freq), str(real), str(img)])
        #self.send_cmd(REPEAT_FREQ)
        self.send_cmd(INCREMENT_FREQ)
        freq += df
    self.send_cmd(POWER_DOWN)

  def make_temp_measure(self):
    self.send_cmd(MEASURE_TEMP)
    while not self.check_status(TEMP_MEAS_STATUS):
      pass
    T = (self.read_reg(TEMP_DATA_REG0) << 8) | self.read_reg(TEMP_DATA_REG1)
    if T < 8192:
      T = T / 32.0
    else:
      T = (T-16384) / 32.0
    return T

  def set_clock(self):
    control_word = self.read_reg(CONTROL_REG1)
    if self.clk == 0:
      self.clk = 16.776    # built-in clock frequency [MHz]
      control_word &= 0b11110111
      print("MCLK set to internal clock.")
    else:
      control_word |= 0b00001000
      print("MCLK set to {} MHz".format(self.clk))
    self.write_reg(CONTROL_REG1, control_word)

  def set_freq_range(self, fo=1e3, df=1e3, steps=10):
    # fo    -- start frequency
    # df    -- frequency incrment
    # steps -- # of increments
    if fo < MIN_FREQ or fo+steps*df > MAX_FREQ or steps > 511:
      print( "Frequencies or number of steps out of range")
    else:
      fo = int((fo / (self.clk*1e6 / 4.0)) * pow(2, 27))
      self.write_reg(FREQ_MIN_REG0, (fo >> 16) & 0xFF)
      self.write_reg(FREQ_MIN_REG1, (fo >> 8 ) & 0xFF)
      self.write_reg(FREQ_MIN_REG2, fo & 0xFF)

      df = int((df / (self.clk*1e6 / 4.0)) * pow(2, 27))
      self.write_reg(FREQ_INC_REG0, (df >> 16) & 0xFF)
      self.write_reg(FREQ_INC_REG1, (df >> 8 ) & 0xFF)
      self.write_reg(FREQ_INC_REG2, df & 0xFF)

      self.write_reg(INC_NUM_REG0, steps >> 8)
      self.write_reg(INC_NUM_REG1, steps & 0xFF)

  def set_PGA_gain(self, gain=1):
    word = self.read_reg(CONTROL_REG0)
    if gain == 1:
      word |= 0b00000001   # 1x gain
    else:
      word &= 0b11111110   # 5x gain
    self.write_reg(CONTROL_REG0, word)

  def set_ex_voltage(self, voltage=2):
    if voltage not in [1,2,3,4]:
      print("Excitation voltage code must be 1-4")
    else:
      bits = EX_VOLTAGE_CODES[voltage-1]
      word = self.read_reg(CONTROL_REG0) & 0b11111001  # clear bits
      word |= bits << 1                                # set bits
      self.write_reg(CONTROL_REG0, word)

  def set_settling_time(self, cy_num, factor=1):
    # cy_num = number of cycles for settling
    # factor multiples cy_num by 1x, 2x, or 4x
    if cy_num > 511 or cy_num < 0 or (factor not in [1,2,4]):
      print("Settling time or scale factor out of allowed range")
    else:
      word = self.read_reg(STTL_TIME_CY_NUM_REG0) & ~1  # clear 1st bit
      if factor == 1:
        word |= 0b00000000
      elif factor == 2:
        word |= 0b00000010
      elif factor == 4:
        word |= 0b00000110
      self.write_reg(STTL_TIME_CY_NUM_REG0, word | cy_num >> 8)
      self.write_reg(STTL_TIME_CY_NUM_REG1, cy_num & 0xFF)

  def check_status(self, state_to_check):
    status = self.read_reg(STATUS_REG)
    return (status & state_to_check)

  def send_cmd(self,cmd):
    word = self.read_reg(CONTROL_REG0)   # get current control state
    word &= 0b1111                       # keep just 4 LSB
    word |= cmd                          # add the command (4 MSB)
    self.write_reg(CONTROL_REG0, word)

  def reset(self):
    word = self.read_reg(CONTROL_REG1)   # get current control state
    word |= RESET                        # add the reset command
    self.write_reg(CONTROL_REG1, word)

  def read_real_reg(self):
    val = (self.read_reg(REAL_DATA_REG0) << 8) | self.read_reg(REAL_DATA_REG1)
    if val & (1 << 15):  # negative value, so convert from twos complement
      val -= pow(2,16)
    return val

  def read_img_reg(self):
    val = (self.read_reg(IMG_DATA_REG0) << 8) | self.read_reg(IMG_DATA_REG1)
    if val & (1 << 15):  # negative value, so convert from twos complement
      val -= pow(2,16)
    return val

  def read_reg(self, reg):
    return self.bus.read_byte_data(self.address, reg)

  def write_reg(self, reg, val):
    self.bus.write_byte_data(self.address, reg, val)

  # utility / debug method to display control and status register contents
  def print_status(self):
    mask = 0b11111111
    print("Status  " + bin(self.read_reg(STATUS_REG) & mask) + '   ' +
          "Control " + bin(self.read_reg(CONTROL_REG0) & mask) + '-' + 
          bin(self.read_reg(CONTROL_REG1) & mask))


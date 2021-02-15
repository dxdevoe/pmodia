from pyAD5933 import AD5933

print("Setting up sensor")
clk = 0    #   clock [MHz] (zero to use internal clock)
port = 1   #   I2C port
dev = 0x0d #   I2C address, check w/ i2cdetect -y 0 (or 1 if port 1)
sensor = AD5933(clk,port,dev)   

print("Measuring temperature")
T = sensor.make_temp_measure()
print("Sensor temperature = {}".format(T))

print("Initializing impedance analyzer")
sensor.init()

#print("Setting gain")
#sensor.set_PGA_gain(1)
#
#print("Setting excitation voltage")
#sensor.set_ex_voltage(4)  # 
#
#print("Setting settling time")
#sensor.set_settling_time(255, 2)

print("Measuring impedance")
fmin = 1e3        # start frequency [Hz]
fmax = 5e3        # end frequency
df = 100         # delta per step 
steps = int(fmax-fmin)/df)       # number of steps
sensor.make_imp_measure(fmin, df, steps)

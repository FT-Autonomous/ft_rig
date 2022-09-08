import subprocess

def get_pinion_distance(i2c_bus=8):
    i2c_bus = str(i2c_bus)
    subprocess.Popen(["i2cset", "-y" ,i2c_bus, "0x52", "0x00", "0x00"]) 
    upper = subprocess.Popen(["i2cget", "-y" ,i2c_bus, "0x52", "0x00"], stdout=subprocess.PIPE).stdout.read()[2:-1]
    lower = subprocess.Popen(["i2cget", "-y" ,i2c_bus, "0x52", "0x01"], stdout=subprocess.PIPE).stdout.read()[2:-1]
    return int(upper + lower, 16)


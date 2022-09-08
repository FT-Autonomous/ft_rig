import subprocess
def get_steering_angle():
    subprocess.Popen(["i2cset", "-y" ,"8", "0x52", "0x00", "0x00"]) 
    upper = subprocess.Popen(["i2cget", "-y" ,"8", "0x52", "0x00"], stdout=subprocess.PIPE).stdout.read()[2:-1]
    lower = subprocess.Popen(["i2cget", "-y" ,"8", "0x52", "0x01"], stdout=subprocess.PIPE).stdout.read()[2:-1]
    return int(upper + lower, 16)

print(get_steering_angle())

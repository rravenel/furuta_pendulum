import unittest
import numpy as np

PI = np.pi
CPR = 8192 # odrive

MOTOR_KV = 1000 # motor spec
TORQUE_FACTOR = 8.3 # iniversal BLDC constant
TORQUE_CONSTANT = TORQUE_FACTOR / MOTOR_KV # multiply this by current to get torque
MAX_CURRENT = 10.0

STEP = 0.01 # time step, seconds

VEL_MAX_ARM = 25 # rad/s
VEL_MAX_POLE = 50 # rad/s

# safety margin
MAX_RADIANS = VEL_MAX_ARM * STEP * 0.8

# 0 is straight up
ANGLE_TERMINAL_MAX_D = 60
ANGLE_TERMINAL_MIN_D = 10

NOISE_SENSE = 0.005
NOISE_COMMAND = 0.05

TRAIN = "train"
RUN = "run"

# clamp(2, 1, -1)
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

# output: -pi <= x <= pi
def rad2Norm(x):
    return (((x+PI) % (2*PI)) - PI)

# shifts range from 0 - +/-pi to 0 - 2pi
def norm2Rad(x):
    return (x + 2*PI) % (2*PI)

# wrap arg around 0 - 2pi
def wrapRad(x):
    return x % (2*PI)

# convert radians to degrees
def rad2Deg(x):
    return (x * (360/(2*PI))) % 360

# output: -180 <= x <= 180
def deg2Norm(x):
    return (((x+180) % (360)) - 180)
    

# convert degrees to radians
def deg2Rad(x):
    return wrapRad((x/180) * PI)

# difference between two positions, accounting for wrap
# always takes the short way around
# order of arguments is order of travel
# difRads(0, PI) = PI
# difRads(PI, 0) = -PI
def difRads(a, b):
    c = b-a
    dir = np.sign(c)
    c *= dir
    
    if c > PI:
        c = -1 * (2*PI - c)
    c *= dir
    
    return c

# convert cpr to radians
def cpr2Rad(c):
    return (c / CPR) * 2 * PI

def rad2Cpr(r):
    r = wrapRad(r)
    return int((r / (2*PI)) * CPR)
    
# dither the val by a random value +/- dith
def dither(val, dith):
    dith = 1 + np.random.uniform(-dith, dith)
    return val * dith

def sgn():
    return [-1,1][np.random.randint(2)]
    

class CommonTest(unittest.TestCase): 
    # numpy.clip() doesn't verify min<max, so neither do we...
    def testClamp(self):
        self.assertEqual(clamp(0, -1, 1), 0)
        self.assertEqual(clamp(1, -1, 1), 1)
        self.assertEqual(clamp(-1, -1, 1), -1)
        self.assertEqual(clamp(2, -1, 1), 1)
        self.assertEqual(clamp(-2, -1, 1), -1)

    def testRad2Norm(self):
        self.assertEqual(round(rad2Norm(.1), 10), round(.1, 10))
        self.assertEqual(round(rad2Norm(.5*PI), 10), round(.5*PI, 10))
        self.assertEqual(round(rad2Norm(PI-.1), 10), round(PI-.1, 10))
        self.assertEqual(round(rad2Norm(PI+.1), 10), round(-PI+.1, 10))
        self.assertEqual(round(rad2Norm(1.5*PI), 10), round(-.5*PI, 10))
        self.assertEqual(round(rad2Norm(2*PI-.1), 10), round(-.1, 10))

        self.assertEqual(round(rad2Norm(-.1), 10), round(-.1, 10))
        self.assertEqual(round(rad2Norm(-.5*PI), 10), round(-.5*PI, 10))
        self.assertEqual(round(rad2Norm(-PI+.1), 10), round(-PI+.1, 10))
        self.assertEqual(round(rad2Norm(-PI-.1), 10), round(PI-.1, 10))
        self.assertEqual(round(rad2Norm(-1.5*PI), 10), round(.5*PI, 10))
        self.assertEqual(round(rad2Norm(-2*PI+.1), 10), round(.1, 10))
    
    def testNorm2Rad(self):
        self.assertEqual(round(norm2Rad(.1), 10), round(.1, 10))
        self.assertEqual(round(norm2Rad(.5*PI), 10), round(.5*PI, 10))
        self.assertEqual(round(norm2Rad(PI-.1), 10), round(PI-.1, 10))
        self.assertEqual(round(norm2Rad(-PI+.1), 10), round(PI+.1, 10))
        self.assertEqual(round(norm2Rad(-.5*PI), 10), round(1.5*PI, 10))
        self.assertEqual(round(norm2Rad(-.1), 10), round(2*PI-.1, 10))
    
    def testWrapRad(self):
        self.assertEqual(round(wrapRad(.1), 10), round(.1, 10))
        self.assertEqual(round(wrapRad(PI), 10), round(PI, 10))
        self.assertEqual(round(wrapRad(2*PI-.1), 10), round(2*PI-.1, 10))
        self.assertEqual(round(wrapRad(2*PI), 10), round(0, 10))
        self.assertEqual(round(wrapRad(2*PI+.1), 10), round(.1, 10))
        
        self.assertEqual(round(wrapRad(-.1), 10), round(2*PI-.1, 10))
        self.assertEqual(round(wrapRad(-PI), 10), round(PI, 10))
        self.assertEqual(round(wrapRad(-2*PI+.1), 10), round(.1, 10))
        self.assertEqual(round(wrapRad(-2*PI), 10), round(0, 10))
        self.assertEqual(round(wrapRad(-2*PI-.1), 10), round(2*PI-.1, 10))
    
    def testRad2Deg(self):
        self.assertEqual(round(rad2Deg(0), 10), round(0, 10))
        self.assertEqual(round(rad2Deg(0.5*PI), 10), round(90, 10))
        self.assertEqual(round(rad2Deg(PI), 10), round(180, 10))
        self.assertEqual(round(rad2Deg(2*PI), 10), round(0, 10))
        self.assertEqual(round(rad2Deg(2.5*PI), 10), round(90, 10))
        self.assertEqual(round(rad2Deg(-.5*PI), 10), round(270, 10))
        self.assertEqual(round(rad2Deg(-PI), 10), round(180, 10))
        self.assertEqual(round(rad2Deg(-2*PI), 10), round(0, 10))
        self.assertEqual(round(rad2Deg(-2.5*PI), 10), round(270, 10))
        
    def testDeg2Rad(self):
        self.assertEqual(round(deg2Rad(0), 10), round(0, 10))
        self.assertEqual(round(deg2Rad(90), 10), round(0.5*PI, 10))
        self.assertEqual(round(deg2Rad(180), 10), round(PI, 10))
        self.assertEqual(round(deg2Rad(270), 10), round(1.5*PI, 10))
        self.assertEqual(round(deg2Rad(360), 10), round(0, 10))
        self.assertEqual(round(deg2Rad(450), 10), round(0.5*PI, 10))
        self.assertEqual(round(deg2Rad(-90), 10), round(1.5*PI, 10))
        self.assertEqual(round(deg2Rad(-180), 10), round(PI, 10))
        self.assertEqual(round(deg2Rad(-270), 10), round(0.5*PI, 10))
        self.assertEqual(round(deg2Rad(-360), 10), round(0, 10))
        self.assertEqual(round(deg2Rad(-450), 10), round(1.5*PI, 10))
  
    def testDifRads(self):
        self.assertEqual(round(difRads(0, 0), 10), round(0, 10))
        self.assertEqual(round(difRads(0, PI), 10), round(PI, 10))
        self.assertEqual(round(difRads(PI, 0), 10), round(-PI, 10))
        self.assertEqual(round(difRads(.5*PI, 1.5*PI), 10), round(PI, 10))
        self.assertEqual(round(difRads(1.5*PI, .5*PI), 10), round(-PI, 10))
        self.assertEqual(round(difRads(.5*PI, 1.5*PI+.1), 10), round(-PI+.1, 10))
        self.assertEqual(round(difRads(1.5*PI+.1, .5*PI), 10), round(PI-.1, 10))
        self.assertEqual(round(difRads(.1, 2*PI-.1), 10), round(-.2, 10))
        self.assertEqual(round(difRads(2*PI-.1, .1), 10), round(.2, 10))
        self.assertEqual(round(difRads(.1, -.1), 10), round(-.2, 10))
        self.assertEqual(round(difRads(-.1, .1), 10), round(.2, 10))
        self.assertEqual(round(difRads(-PI, PI), 10), round(0, 10))
        self.assertEqual(round(difRads(-.5*PI, .5*PI), 10), round(PI, 10))
        self.assertEqual(round(difRads(.5*PI, -.5*PI), 10), round(-PI, 10))
        self.assertEqual(round(difRads(.5*PI, -.5*PI+.1), 10), round(-PI+.1, 10))
        self.assertEqual(round(difRads(-.5*PI, .5*PI-.1), 10), round(PI-.1, 10))
    
    def testCpr2Rad(self):
        self.assertEqual(round(cpr2Rad(0), 10), round(0, 10))
        self.assertEqual(round(cpr2Rad(CPR/2), 10), round(PI, 10))
        self.assertEqual(round(cpr2Rad(CPR), 10), round(2*PI, 10))
    
    def testRad2Cpr(self):
        self.assertEqual(round(rad2Cpr(0), 10), round(0, 10))
        self.assertEqual(round(rad2Cpr(PI), 10), round(CPR/2, 10))
        self.assertEqual(round(rad2Cpr(2*PI), 10), round(0, 10))
        self.assertEqual(round(rad2Cpr(-PI), 10), round(CPR/2, 10))
        self.assertEqual(round(rad2Cpr(3*PI), 10), round(CPR/2, 10))
        self.assertEqual(round(rad2Cpr(-3*PI), 10), round(CPR/2, 10))
        
if __name__ == '__main__': 
    unittest.main() 
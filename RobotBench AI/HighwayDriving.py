"""Sample Webots controller for highway driving benchmark."""

from vehicle import Driver
import numpy as np
import math


def mean(list):
    s = 0
    for elem in list:
        s += elem
    return s/len(list)

# name of the available distance sensors
sensorsNames = [
    'front',
    'front right 0',
    'front right 1',
    'front right 2',
    'front left 0',
    'front left 1',
    'front left 2',
    'rear',
    'rear left',
    'rear right',
    'right',
    'left']
sensors = {}

maxSpeed = 80
driver = Driver()
driver.setSteeringAngle(0.0)  # go straight

# get and enable the distance sensors
for name in sensorsNames:
    sensors[name] = driver.getDistanceSensor('distance sensor ' + name)
    sensors[name].enable(10)

# get and enable the GPS
gps = driver.getGPS('gps')
gps.enable(10)

# get the camera
#camera = driver.getCamera('camera')
# uncomment those lines to enable the camera
# camera.enable(50)
# camera.recognitionEnable(50)

K = np.array([[-0.5, 2.5, 5, 7.5, 15, -7.5, -5, -2.5],
              [0.5, -2.5, -5, -7.5, -15, 7.5, 5, 2.5]])

MV = np.array([[maxSpeed], [maxSpeed]])
goalLeftDist = 1.5

while driver.step() != -1:
    # get sensor reads
    frontDistance = sensors['front'].getValue()
    frontRange = sensors['front'].getMaxValue()
    fr0Distance = sensors['front right 0'].getValue()
    fr0Range = sensors['front right 0'].getMaxValue()
    fr1Distance = sensors['front right 1'].getValue()
    fr1Range = sensors['front right 1'].getMaxValue()
    fr2Distance = sensors['front right 2'].getValue()
    fr2Range = sensors['front right 2'].getMaxValue()
    fl0Distance = sensors['front left 0'].getValue()
    fl0Range = sensors['front left 0'].getMaxValue()
    fl1Distance = sensors['front left 1'].getValue()
    fl1Range = sensors['front left 1'].getMaxValue()
    fl2Distance = sensors['front left 2'].getValue()
    fl2Range = sensors['front left 2'].getMaxValue()
    rightDistance = sensors['right'].getValue()
    rightRange = sensors['right'].getMaxValue()
    leftDistance = sensors['left'].getValue()
    leftRange = sensors['left'].getMaxValue()

    frontLeft0 = 1-fl0Distance/fl0Range
    frontLeft1 = 1-fl1Distance/fl1Range
    frontLeft2 = 1-fl2Distance/fl2Range
    frontRight0 = 1-fr0Distance/fr0Range
    frontRight1 = 1-fr1Distance/fr1Range
    frontRight2 = 1-fr2Distance/fr2Range
    central = (frontDistance < 10)*(1-frontDistance/frontRange)
    left = leftDistance-goalLeftDist


    S = np.array([[left], [frontLeft2], [frontLeft1], [frontLeft0], [central], [frontRight0], [frontRight1], [frontRight2]])
    V = MV + np.matmul(K,S)
    ML = V.item(0)
    MR = V.item(1)
    ML = min(max(ML,0),maxSpeed)
    MR = min(max(MR,0),maxSpeed)
    print(S)
    # calculate speed and steering angle
    speed = mean([ML, MR])
    speed = min(speed, speed*frontDistance/frontRange)
    steer = (ML-MR)*1/100 # [-80,80] -> [-0.4,0.4]
    driver.setSteeringAngle(steer)
    driver.setCruisingSpeed(speed)


    speedDiff = driver.getCurrentSpeed() - speed
    if speedDiff > 0:
        driver.setBrakeIntensity(min(speedDiff / speed, 1))
    else:
        driver.setBrakeIntensity(0)

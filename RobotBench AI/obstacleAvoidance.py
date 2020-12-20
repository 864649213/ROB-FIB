"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot
import numpy as np

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 360

# Get left and right wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

# Get frontal distance sensors.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = 0.7 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

# ----------------------------------------------------------------------------------------------
ST_weight = 0.02;

# Sensor weight to the motors 
K = np.array([[-0.75, -1.5, -2, 1.5, 0.75, ST_weight],
              [0.75, 1.5, 2, -1.5, -0.75, -ST_weight]])

# Maximum velocity of the motor
MV = np.array([[maxMotorVelocity], [maxMotorVelocity]])

# Deviation of orientation to frontal axis
ST = 0

while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant

    S = np.array([[outerLeftSensorValue], [centralLeftSensorValue], [centralSensorValue], [centralRightSensorValue],
                    [outerRightSensorValue], [ST]])

    print(S)
 
    V = MV - np.matmul(K,S) 
    ML = V.item(0)
    MR = V.item(1)
    ST = ST + (ML-MR)*0.1

    # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
    leftMotor.setVelocity(ML)
    rightMotor.setVelocity(MR)
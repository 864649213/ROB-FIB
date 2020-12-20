"""Sample Webots controller for the wall following benchmark."""

from controller import Robot
import numpy as np
import math



def getDistance(sensor):
    """
    Return the distance of an obstacle for a sensor.

    The value returned by the getValue() method of the distance sensors
    corresponds to a physical value (here we have a sonar, so it is the
    strength of the sonar ray). This function makes a conversion to a
    distance value in meters.
    """
    return ((1000 - sensor.getValue()) / 1000) * 5


# Maximum speed for the velocity value of the wheels.
# Don't change this value.
MAX_SPEED = 5.24

# Get pointer to the robot.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get pointer to the robot wheels motors.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

# We will use the velocity parameter of the wheels, so we need to
# set the target position to infinity.
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

# Get and enable the distance sensors.
fSensor = robot.getDistanceSensor("so3")
fSensor.enable(timestep)

fd2Sensor = robot.getDistanceSensor("so2")
fd2Sensor.enable(timestep)
fd1Sensor = robot.getDistanceSensor("so1")
fd1Sensor.enable(timestep)


flSensor = robot.getDistanceSensor("so0")
flSensor.enable(timestep)

blSensor = robot.getDistanceSensor("so15")
blSensor.enable(timestep)




# Move forward until we are 50 cm away from the wall.
leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(MAX_SPEED)
while robot.step(timestep) != -1:
    if getDistance(fSensor) < 0.5:
        break

# Rotate clockwise until the wall is to our left.
leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(-MAX_SPEED)
while robot.step(timestep) != -1:
    # Rotate until there is a wall to our left, and nothing in front of us.
    if getDistance(flSensor) < 1:
        break

# --------------------------------------------------------------------------------------------


# Sensor weight to the motors
K = np.array([[5, -10, -20, -20, -5],
              [-5, 10, 20, 20, 5]])

# Maximum velocity of the motor
MV = np.array([[MAX_SPEED], [MAX_SPEED]])

# trigonometry: sin(90-ang)=0.5/h => h=0.5/sin(90-ang)
goalDistWithWall = 0.5
diag1GoalDist = goalDistWithWall/math.sin(math.radians(50))
diag2GoalDist = goalDistWithWall/math.sin(math.radians(70))
frontGoalDist = goalDistWithWall/math.sin(math.radians(80))
maxSensor = 5 #max sensor range

# Main loop.
while robot.step(timestep) != -1:

    flD = abs(getDistance(flSensor))
    blD = abs(getDistance(blSensor))
    frontD = abs(getDistance(fSensor))
    diag1D = abs(getDistance(fd1Sensor))
    diag2D = abs(getDistance(fd2Sensor))

    wall = min(flD-goalDistWithWall, goalDistWithWall)
    sideDiff = max(min(blD-flD, 0.5), -0.5)
    diag1 = max(diag1GoalDist-diag1D, 0)
    diag2 = max(diag2GoalDist-diag2D, 0)
    front = max(frontGoalDist-frontD, 0)


    S = np.array([[wall], [diag1], [diag2], [front], [sideDiff]])
    V = MV - np.matmul(K,S)
    print(S)
    print("------------------------------------")

    ML = min(max(V.item(0),0), MAX_SPEED) # can't pass speed limit
    MR = min(max(V.item(1),0), MAX_SPEED)

    leftWheel.setVelocity(ML)
    rightWheel.setVelocity(MR)



# Stop the robot when we are done.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)

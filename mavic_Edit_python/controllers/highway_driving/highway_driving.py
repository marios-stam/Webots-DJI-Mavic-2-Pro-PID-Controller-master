"""Sample Webots controller for highway driving benchmark."""

from vehicle import Driver

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

maxSpeed = 20
driver = Driver()
driver.setSteeringAngle(0.0)  # go straight



while driver.step() != -1:
    t=driver.getTime()
    if(t>6):
        speed=5
        driver.setCruisingSpeed(speed)
    

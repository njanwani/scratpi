#!/usr/bin/env python3
#
#   Drive the Robot
#
import sys
import time
import qwiic_scmd


# Define the constants.
L_MTR = 1
R_MTR = 0
REVERSE_L = 1
REVERSE_R = 1


#
#   Drive Function
#
def driveBot(driver):
    # Set speed to zero to enable.
    print('Enabling');
    driver.set_drive(L_MTR, REVERSE_L, 0)
    driver.set_drive(R_MTR, REVERSE_R, 0)
    driver.enable()

    # Wait.
    time.sleep(0.25)

    # Set speed to something.
    print('Driving both positive');
    driver.set_drive(L_MTR, REVERSE_L, 110)
    driver.set_drive(R_MTR, REVERSE_R, 110)

    # Wait.
    time.sleep(1.0)

    # Set speed to zero.
    print('Stopping');
    driver.set_drive(L_MTR, REVERSE_L, 0)
    driver.set_drive(R_MTR, REVERSE_R, 0)

    # Wait.
    time.sleep(0.25)

    # Set speed to zero.
    print('Disabling');
    driver.disable()


#
#   Main
#
if __name__ == "__main__": 
    # Establish a connection to the motor drivers.
    driver = qwiic_scmd.QwiicScmd()

    # Check.
    if driver.connected == False:
        print("Motor Driver not connected!")
        sys.exit(0)

    # Initialize.
    driver.begin()
    print("Motor driver initialized.")

    # Run while watching exceptions.
    try:
        driveBot(driver)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        driver.disable()
        sys.exit(0)

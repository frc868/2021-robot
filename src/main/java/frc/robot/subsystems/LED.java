/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * All of this is untested.
 * Proves and addresses the LED strip connected to the roboRIO via PWM.
 * @author ama
 */
public class LED {
    private static LED instance;

    private AddressableLED leds;
    private AddressableLEDBuffer data;

    /**
     * Makes new instance of Leds.
     */
    private LED() {
        leds = new AddressableLED(RobotMap.LED.PORT);
        data = new AddressableLEDBuffer(RobotMap.LED.LENGTH);

        leds.setLength(data.getLength());
    }

    /**
     * Returns a singleton instance of the LED class.
     */
    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }
        return instance;
    }

    /**
     * Updates the current state of the LED strip to a solid color.
     * @param r the red value from 0-255
     * @param g the green value from 0-255
     * @param b the blue value from 0-255
     */
    public void solidColor(int r, int g, int b) {
        for (int i = 0; i < data.getLength(); i++) {
            data.setRGB(i, r, g, b);
        }
        leds.setData(data);
        leds.start();
    }

    /**
     * Updates the current state of the LED strip to a certain color.
     * It does this based on the number of balls the robot currently has.
     * @author gra, hrl
     */
    public void colorInventory() {
        switch (Robot.hopper.getBallCount()) {
            case 0:
                solidColor(255, 255, 255);
                break;
            case 1:
                solidColor(255, 0, 0);
                break;
            case 2:
                solidColor(255, 40, 0);
                break;
            case 3:
                solidColor(255, 255, 0);
                break;
            case 4:
                solidColor(0, 0, 255);
                break;
            case 5:
                solidColor(255, 0, 255);
                break;
            default:
                solidColor(0, 0, 0);
                break;
        }
    }
}

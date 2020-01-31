/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotMap;

/**
 * Proves and addresses the LED strip connected to the roboRIO via PWM.
 * 
 * @author jml
 */
public class LED {
    private static LED instance;

    private AddressableLED leds;
    private AddressableLEDBuffer data;

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
     * Updates the current state of the LED strip to a solid blue.
     */
    public void solidColor() {
        for (int i = 0; i < data.getLength(); i++) {
            data.setRGB(i, 0, 0, 255);
        }
        leds.setData(data);

        leds.start();
    }

    /**
     * Updates the current state of the LED strip to a rainbow pattern.
     */
    

    public void colorInventory() {
        int cellCount = 5;

if(cellCount == 1){
for (var i = 0; i < data.getLength(); i++) {
   // Sets the specified LED to the RGB values for red
   data.setRGB(i, 255, 255, 255);
}leds.setData(data);
}else if(cellCount == 2){
for (var i = 0; i < data.getLength(); i++) {
   // Sets the specified LED to the RGB values for red
   data.setRGB(i, 255, 0, 255);
}
leds.setData(data);
}else if(cellCount == 3){
for (var i = 0; i < data.getLength(); i++) {
   // Sets the specified LED to the RGB values for red
data.setRGB(i, 0, 0, 255);
}
leds.setData(data);
}else if(cellCount == 4){
for (var i = 0; i < data.getLength(); i++) {
   // Sets the specified LED to the RGB values for red
   data.setRGB(i, 255, 40, 0);
}
leds.setData(data);
}else if(cellCount == 5){
for (var i = 0; i < data.getLength(); i++) {
   // Sets the specified LED to the RGB values for red
   data.setRGB(i, 255, 255, 0);
}
leds.setData(data);
}

    

}
}

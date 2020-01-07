package frc.robot.subsystems;

/** 
 * This is the code for the Wheel of Fortune manipulator. It initializes motor controllers and has
 * methods for its various functions.
 * @author 
 */
public class WheelOfFortune {
    private static WheelOfFortune instance;

    private WheelOfFortune() {

    }

    public static WheelOfFortune getInstance() {
        if (instance == null) {
            return new WheelOfFortune();
        }
        return instance;
    }
}
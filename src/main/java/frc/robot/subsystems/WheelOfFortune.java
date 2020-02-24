package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * This is the code for the Wheel of Fortune manipulator. It initializes motor
 * controllers and has methods for its various functions.
 *
 * @author dri, ai, gjs, hrl
 */
public class WheelOfFortune {
    private static WheelOfFortune instance;

    private enum WOFColor {
        RED, GREEN, BLUE, YELLOW, NONE;
    }

    private WPI_TalonSRX primary;
    private DoubleSolenoid actuator;

    private ColorSensorV3 colorSensor;
    private ColorMatch colorMatch;
    private Color detectedColor;

    // hbd
    private final Color redTarget = ColorMatch.makeColor(RobotMap.WheelOfFortune.RED_VALUES[0],
                                                         RobotMap.WheelOfFortune.RED_VALUES[1],
                                                         RobotMap.WheelOfFortune.RED_VALUES[2]);
    private final Color greenTarget = ColorMatch.makeColor(RobotMap.WheelOfFortune.GREEN_VALUES[0],
                                                           RobotMap.WheelOfFortune.GREEN_VALUES[1],
                                                           RobotMap.WheelOfFortune.GREEN_VALUES[2]);
    private final Color blueTarget = ColorMatch.makeColor(RobotMap.WheelOfFortune.BLUE_VALUES[0],
                                                          RobotMap.WheelOfFortune.BLUE_VALUES[1],
                                                          RobotMap.WheelOfFortune.BLUE_VALUES[2]);
    private final Color yellowTarget = ColorMatch.makeColor(RobotMap.WheelOfFortune.YELLOW_VALUES[0],
                                                            RobotMap.WheelOfFortune.YELLOW_VALUES[1],
                                                            RobotMap.WheelOfFortune.YELLOW_VALUES[2]);

    private WheelOfFortune() {
        primary = new WPI_TalonSRX(RobotMap.WheelOfFortune.MOTOR);
        actuator = new DoubleSolenoid(RobotMap.WheelOfFortune.ACTUATOR1,
                                      RobotMap.WheelOfFortune.ACTUATOR2);

        colorSensor = new ColorSensorV3(RobotMap.WheelOfFortune.COLOR_SENSOR);
        colorMatch = new ColorMatch();

        colorMatch.addColorMatch(blueTarget);
        colorMatch.addColorMatch(greenTarget);
        colorMatch.addColorMatch(redTarget);
        colorMatch.addColorMatch(yellowTarget);
    }

    /**
     * Checks to see if the instance of this class has already been created.
     * If so, return it. If not, create it and return it.
     * @return instance of the WheelOfFortune class
     */
    public static WheelOfFortune getInstance() {
        if (instance == null) {
            return new WheelOfFortune();
        }
        return instance;
    }

    /**
     * Sets the speed of the mechanism.
     * @param speed the speed from -1 to 1
     */
    public void setSpeed(double speed) {
        primary.set(speed);
    }

    /**
     * takes game data sent by the FMS and accordingly returns the Color which we need to target
     * @param gameData the data sent by the FMS
     * @return WOFColor enum for the sensor to target
     */
    private WOFColor getPCValue(String gameData) {
        WOFColor colorPassed = WOFColor.NONE;

        if (gameData.equals("B")) {
            colorPassed = WOFColor.BLUE;
        } else if (gameData.equals("G")) {
            colorPassed = WOFColor.GREEN;
        } else if (gameData.equals("R")) {
            colorPassed = WOFColor.RED;
        } else if (gameData.equals("Y")) {
            colorPassed = WOFColor.YELLOW;
        }

        return colorPassed;
    }

    /**
     * Returns the output power scaled to avoid overshoot
     * @param numColors the number of colors to go over
     */
    private double getOutputPower(double numColors) {
        if (numColors > 32) {
            return 0.15;
        } else {
            return 0.03*Math.log(numColors + 1)+0.05;
        }
    }

    /**
     * Runs color-matching on the currently detected color
     * @return a WOFColor representing the current color
     */
    public WOFColor colorDetect() {
        detectedColor = colorSensor.getColor();

        WOFColor ret = WOFColor.NONE;

        ColorMatchResult match = colorMatch.matchColor(detectedColor);
        if (match == null) {
            ret = WOFColor.NONE;
        } else if (match.color == blueTarget) {
            ret = WOFColor.BLUE;
        } else if (match.color == redTarget) {
            ret = WOFColor.RED;
        } else if (match.color == greenTarget) {
            ret = WOFColor.GREEN;
        } else if (match.color == yellowTarget) {
            ret = WOFColor.YELLOW;
        }

        return ret;
    }

    /**
     * Returns the desired amount of rotations for a given start and end color.
     * @param startColor a WOFColor representing the color we are currently on
     * @param goalColor a WOFColor representing the color we want to go to
     * @return a number between 0 and 3 as to how far we should be turning
     */
    public int calculateDesiredRotations(WOFColor startColor, WOFColor goalColor) {
        switch (startColor) {
        case BLUE:
            switch (goalColor) {
            case BLUE:          return 0;
            case YELLOW:        return 1;
            case RED:           return 2;
            case GREEN:         return 3;
            case NONE: default: return 0;
            }
        case GREEN:
            switch (goalColor) {
            case GREEN:         return 0;
            case BLUE:          return 1;
            case YELLOW:        return 2;
            case RED:           return 3;
            case NONE: default: return 0;
            }
        case RED:
            switch (goalColor) {
            case RED:           return 0;
            case GREEN:         return 1;
            case BLUE:          return 2;
            case YELLOW:        return 3;
            case NONE: default: return 0;
            }
        case YELLOW:
            switch (goalColor) {
            case YELLOW:        return 0;
            case RED:           return 1;
            case GREEN:         return 2;
            case BLUE:          return 3;
            case NONE: default: return 0;
            }
        case NONE:
        default:
            return 0;
        }
    }

    /**
     * sets the initial color read by the sensor for position control
     * can be adjusted in testing
     * @return the initial Color
     */
    public WOFColor setInitialColor() {
        return colorDetect();
    }

    /**
     * makes the wheel spin 4 times for rotation control of the color wheel
     * @param initialColor the result of the setInitialColor() method (the color we start on)
     * @param numColorChange the number of color changes to spin. 3 times is 32.
     */
    public void rotationControl(WOFColor initialColor, int numColorChange) {
        // read values of color sensor every 20 ms to check if matches original color
        primary.set(this.getOutputPower(numColorChange));
        int colorChangeCount = 0;

        WOFColor previousColor = initialColor;

        while (colorChangeCount < numColorChange) {
            WOFColor currentColor = colorDetect();
            if ((previousColor != currentColor) && (currentColor != WOFColor.NONE)) {
                colorChangeCount++;
                previousColor = currentColor;

                // set the motor output power, scaled
                primary.set(this.getOutputPower(numColorChange - colorChangeCount));
            }
        }

        primary.set(0);
    }

    /**
     * makes the wheel spin to the color from the Smart Dashboard or FMS
     */
    public void spinToWin() {
        WOFColor aimColor = getPCValue(DriverStation.getInstance().getGameSpecificMessage());
        WOFColor detectedColor = colorDetect();
        switch (aimColor) {
        case RED:
            detectedColor = colorDetect();
            rotationControl(detectedColor, calculateDesiredRotations(detectedColor, WOFColor.RED));
            stop();
            break;
        case BLUE:
            detectedColor = colorDetect();
            rotationControl(detectedColor, calculateDesiredRotations(detectedColor, WOFColor.BLUE));
            stop();
            break;
        case GREEN:
            detectedColor = colorDetect();
            rotationControl(detectedColor, calculateDesiredRotations(detectedColor, WOFColor.GREEN));
            stop();
            break;
        case YELLOW:
            detectedColor = colorDetect();
            rotationControl(detectedColor, calculateDesiredRotations(detectedColor, WOFColor.YELLOW));
            stop();
            break;
        case NONE:
        default:
            System.out.println("NONE");
            break;
        }
    }

    /**
     * gets rgb values for smart dashboard
     * @return rgb values from detected color
     */
    public Color getDetectedColor() {
        double red = colorSensor.getColor().red;
        double green = colorSensor.getColor().green;
        double blue = colorSensor.getColor().blue;

        return ColorMatch.makeColor(red, green, blue); // TODO: evaluate use of this method
    }

    /**
     * stops the motor rotation
     */
    public void stop() {
        primary.set(0);
    }

    /**
     * takes a string from the SmartDashboard and sets the target.
     */
    public static String getTargetColor(){
        String targetColor =  SmartDashboard.getString("Color", "B");
        SmartDashboard.putString("Color", targetColor);
        return targetColor;
    }

    /**
     * sets the actuator to the position it is not currently in
     * @author hrl, acr
     */
    public void toggle() {
        if (actuator.get() == Value.kForward) {
            actuator.set(Value.kReverse);
        } else if (actuator.get() == Value.kReverse) {
            actuator.set(Value.kForward);
        }
    }

    /**
     * Raise the arm.
     * @author acr
     */
    public void actuatorUp() {
        actuator.set(Value.kReverse);
    }

    /**
     * Lower the arm.
     * @author acr
     */
    public void actuatorDown() {
        actuator.set(Value.kForward);
    }

    /**
     * update values on the SmartDashboard
     */
    public void updateSD() {
        SmartDashboard.putString("Detected color", Robot.wheel.getDetectedColor().toString());
    }
}

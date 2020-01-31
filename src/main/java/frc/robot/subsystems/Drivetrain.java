package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.OI;
import frc.robot.RobotMap;

/** 
 * This is the code for the robot drivetrain. It initializes motor controllers and has methods
 * for various functions of the drivetrain.
 * @author gjs
 */
public class Drivetrain {
    private CANSparkMax l_primary, l_secondary, r_primary, r_secondary; 
    private static Drivetrain instance;
    private SpeedControllerGroup leftSpeedControl;
    private SpeedControllerGroup rightSpeedControl;

    private final double INCHES_PER_TICK = 1; // TODO: entirely untested!

    private Drivetrain() {
        l_primary = new CANSparkMax(RobotMap.Drivetrain.LEFT_PRIMARY, MotorType.kBrushless);
        r_primary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_PRIMARY, MotorType.kBrushless);
        l_secondary = new CANSparkMax(RobotMap.Drivetrain.LEFT_PRIMARY, MotorType.kBrushless);
        r_secondary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_SECONDARY, MotorType.kBrushless);

        leftSpeedControl = new SpeedControllerGroup(l_primary,l_secondary);
        rightSpeedControl = new SpeedControllerGroup(r_primary, r_secondary);

        leftSpeedControl.setInverted(RobotMap.Drivetrain.LEFT_IS_INVERTED);
        rightSpeedControl.setInverted(RobotMap.Drivetrain.RIGHT_IS_INVERTED);
    }

    /**
     * creates a new instance of the drivetrain class if one has not been made
     * @return an instance of the drivetrain class
     */
    public static Drivetrain getInstance() {
        if (instance == null) {
            return new Drivetrain();
        }
        return instance;
    }

    /**
     * Sets the left speed of the drivetrain
     * @param speed tbe speed to set from -1 to 1
     */
    public void setLeftSpeed(double speed){
        leftSpeedControl.set(speed);
    }

    /**
     * Sets the right side speed of the drivetrain.
     * @param speed the speed to set to from -1 to 1
     */
    public void setRightSpeed(double speed){
        rightSpeedControl.set(speed);
    }

    /**
     * Maps joysticks to the drivetrain for Arcade layout
     * @param speed scaling factor for robot speed
     */
    public void arcadeDrive(double speed){
        double y = OI.driver.getLY();
        double x = OI.driver.getRX();
        y = -1 * y * speed;
        x = 1 * x * speed;
        setSpeed(x+y, x-y);
    }

    /**
     * Sets the speed of both the control groups
     * @param leftSpeed speed of the left side of the drivetrain from -1 to 1
     * @param rightSpeed speed of the right side of the drivetrain from -1 to 1
     */
    public void setSpeed(double leftSpeed, double rightSpeed) {
        setRightSpeed(rightSpeed);
        setLeftSpeed(leftSpeed);
    }

    /**
     * drives straight using a P controller
     * @param targetDist the distance you want the robot to travel
     * @param startPower the starting power 
     * @param endPower the ending power
     */
    public void driveStraight(double targetDist, double startPower, double endPower) {
        double pGain = 0.5; // TODO: check this constant
        double initialDist = getAveragePosition();
        double distanceToTarget = Math.abs(targetDist) - Math.abs(getAveragePosition() - initialDist);

        double targetSpeed = pGain * (startPower + ((endPower - startPower) / distanceToTarget));

        if (distanceToTarget > 0) {
            setSpeed(targetSpeed, targetSpeed); // TODO: code sanity check
        }
    }

    /**
     * drives in an arc
     * loosely adapted from 2018-robot
     * @param rightInitial the current (at the beginning of driveArc) drivetrain position for the right side
     * @param leftInitial the current (at the beginning of driveArc) drivetrain position for the left side
     * @param rightInches the number of inches to move the right side of the drivetrain
     * @param leftInches the number of inches to move the left side of the drivetrain
     * @param rightPower the power to set the right side of the drivetrain to
     * @param leftPower the power to set the left side of the drivetrain to
     */
    public void driveArc(double rightInitial, double leftInitial, double rightInches, double leftInches, double rightPower, double leftPower) {
        double setRight = rightPower;
        double setLeft = leftPower;

        // is the right side of the drivetrain finished moving?
        if (Math.abs(this.getRightPosition() - rightInitial) > Math.abs(rightInches)) {
            setRight = 0;
        }

        // is the left side of the drivetrain finished moving?
        if (Math.abs(this.getLeftPosition() - leftInitial) > Math.abs(leftInches)) {
            setLeft = 0;
        }
        
        this.setSpeed(setLeft, setRight);
    }

    /**
     * gets the right side primary motor's encoder position
     * @return right encoder position
     * @author dri
     */
    public double getRightPosition() {
        return r_primary.getEncoder().getPosition() * INCHES_PER_TICK;
    }

    /**
     * gets the left side primary motor's encoder position
     * @return left encoder position
     * @author dri
     */
    public double getLeftPosition() {
        return l_primary.getEncoder().getPosition() * INCHES_PER_TICK;
    }
    
    /**
     * gets the average encoder position of the drivetrain
     * @return average encoder position
     * @author dri
     */
    public double getAveragePosition() {
        return (l_primary.getEncoder().getPosition() + r_primary.getEncoder().getPosition())/2;
    }
}

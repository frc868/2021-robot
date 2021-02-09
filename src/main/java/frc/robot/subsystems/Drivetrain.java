package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.OI;
import frc.robot.Robot;
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
    private PIDController leftPID;
    private double klP = 0.0;
    private double klI = 0.0;
    private double klD = 0.0;

    private double initialDistance = 0; // used for driveStraight()

    private final double INCHES_PER_TICK = 1; // TODO: entirely untested!

    private Drivetrain() {
        l_primary = new CANSparkMax(RobotMap.Drivetrain.LEFT_PRIMARY, MotorType.kBrushless);
        r_primary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_PRIMARY, MotorType.kBrushless);
        l_secondary = new CANSparkMax(RobotMap.Drivetrain.LEFT_SECONDARY, MotorType.kBrushless);
        r_secondary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_SECONDARY, MotorType.kBrushless);

        leftSpeedControl = new SpeedControllerGroup(l_primary,l_secondary);
        rightSpeedControl = new SpeedControllerGroup(r_primary, r_secondary);
        leftPID = new PIDController(klP, klI, klD);
        leftSpeedControl.setInverted(RobotMap.Drivetrain.LEFT_IS_INVERTED);
        rightSpeedControl.setInverted(RobotMap.Drivetrain.RIGHT_IS_INVERTED);

        l_primary.getEncoder()
            .setPositionConversionFactor(INCHES_PER_TICK); // set scale for encoder ticks
        r_primary.getEncoder()
            .setPositionConversionFactor(INCHES_PER_TICK);
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
        y = -speed * y;
        x = -speed * x;
        setSpeed(y-x, y+x);
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
     * @author hrl
     */
    public void driveStraight(double targetDist, double startPower, double endPower) {
        if (this.initialDistance == 0) {
            this.initialDistance = Math.abs(getLeftPosition());
        }

        double pGain = 0.5; // TODO: untested
        double distanceToTarget = Math.abs(targetDist) - Math.abs(getLeftPosition() - this.initialDistance);

        double targetSpeed = pGain * (startPower + ((endPower - startPower) / distanceToTarget));
        setSpeed(targetSpeed, targetSpeed);

        distanceToTarget = Math.abs(targetDist) - Math.abs(getLeftPosition() - this.initialDistance);
    }

    /**
     * resets the initial distance used by driveStraight()
     * @author hrl
     */
    public void resetInitialDistance() {
        this.initialDistance = 0;
    }

    /**
     * returns the current distance traversed relative to the initial distance
     * @author hrl
     */
    public double getCurrentDistance() {
        // ensure no races
        if (this.initialDistance == 0) {
            this.initialDistance = Math.abs(getLeftPosition());
        }

        return Math.abs(this.getLeftPosition() - this.initialDistance);
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
     * @author hrl
     */
    public void driveArc(double rightInitial, double leftInitial, double rightInches,
                         double leftInches, double rightPower, double leftPower) {
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
        return r_primary.getEncoder().getPosition();
    }

    /**
     * gets the left side primary motor's encoder position
     * @return left encoder position
     * @author dri
     */
    public double getLeftPosition() {
        return l_primary.getEncoder().getPosition();
    }

    /**
     * gets the average encoder position of the drivetrain
     * @return average encoder position
     * @author dri
     */
    public double getAveragePosition() {
        return (l_primary.getEncoder().getPosition() + r_primary.getEncoder().getPosition())/2;
    }

    /**
     * resets both of the encoder positions
     * @author hrl
     */
    public void resetEncoderPositions() {
        // this has a time delay. this is a hack. i hate it. but this is how it's
        // going to go down.
        l_primary.getEncoder().setPosition(0);
        r_primary.getEncoder().setPosition(0);

        Timer timer = new Timer();
        timer.reset();
        timer.start();

        while (timer.get() < 0.2) {
            // do nothing
        }
        timer.stop();
    }

    @Override
    public String toString() {
        return "" + getAveragePosition();
    }
    public void turnLeft(){
        Robot.drivetrain.resetEncoderPositions();
        if(Robot.gyro.getAngle() < 90){
            double pidOut = leftPID.calculate(Robot.drivetrain.getCurrentDistance(), 0);
            Robot.drivetrain.setRightSpeed(pidOut);
        }
    }
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.OI;
import frc.robot.RobotMap;

/** 
 * This is the code for the robot drivetrain. It initializes motor controllers and has methods
 * for various functions of the drivetrain.
 * @author ya boi gus
 */
public class Drivetrain {
    private CANSparkMax l_primary, l_secondary, r_primary, r_secondary; 
    private static Drivetrain instance;
    private SpeedControllerGroup leftSpeedControl;
    private SpeedControllerGroup rightSpeedControl; 

    private Drivetrain() {
        l_primary = new CANSparkMax(RobotMap.Drivetrain.LEFT_PRIMARY, MotorType.kBrushless);
        r_primary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_PRIMARY, MotorType.kBrushless);
        l_secondary = new CANSparkMax(RobotMap.Drivetrain.LEFT_PRIMARY, MotorType.kBrushless);
        r_secondary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_SECONDARY, MotorType.kBrushless);

        leftSpeedControl = new SpeedControllerGroup(l_primary,l_secondary);
        rightSpeedControl = new SpeedControllerGroup(r_primary, r_secondary);

        leftSpeedControl.setInverted(false); //TODO: untested, please test when given robot
        rightSpeedControl.setInverted(false); //TODO: ditto as above, MUST TEST
    }
    public void setLeftSpeed(double speed){
        leftSpeedControl.set(speed);
    }
    public void setRightSpeed(double speed){
        rightSpeedControl.set(speed);
    }
    public void arcadeDrive(double speed){
        double y = OI.driver.getLY();
        double x = OI.driver.getRX();
        y = -1 * y * speed; //TODO equation untested
        x = 1 * x * speed; //TODO equation untested
        setSpeed(x+y, x-y); // TODO equation untested
    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            return new Drivetrain();
        }
        return instance;
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        setRightSpeed(rightSpeed);
        setLeftSpeed(leftSpeed);
    }

    public void driveStraight(double targetDist, double startPower, double endPower) {
        double pGain = 0.5;
        double initialDist = getAveragePosition();
        double distanceToTarget = Math.abs(targetDist) - Math.abs(getAveragePosition() - initialDist);

        double targetSpeed = pGain * (startPower + ((endPower - startPower) / distanceToTarget));

        if (distanceToTarget > 0) {
            setSpeed(targetSpeed, targetSpeed); // TODO: code sanity check
        }
    }
    
    public double getAveragePosition() {
        return 0; // TODO: implement this method
    }
}
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.RobotMap;

/**
 * This is the code for the climber. It initializes motor controllers/pneumatics
 * and has methods for various functions of the climber.
 * 
 * @author
 */
public class Climber {
    private static Climber instance;
    private CANSparkMax primary_winch, secondary_winch, arm;
    public double kP_arm, kP_winch, kI_arm, kI_winch, kD_arm, kD_winch, kMaxOutput, kMinOutput, maxRPM, kFF_arm, kFF_winch, kIz_arm, kIz_winch;
    private CANPIDController pidControllerArm, pidControllerWinch;
    private double initialPosition;
    private DoubleSolenoid actuator;
    private static boolean engaged;

    private Climber() {
        primary_winch = new CANSparkMax(RobotMap.Climber.PRIMARY_WINCH, MotorType.kBrushless);
        secondary_winch = new CANSparkMax(RobotMap.Climber.SECONDARY_WINCH, MotorType.kBrushless);
        arm = new CANSparkMax(RobotMap.Climber.ARM, MotorType.kBrushless);
        secondary_winch.follow(primary_winch);
        actuator = new DoubleSolenoid(RobotMap.Climber.ACTUATOR1, RobotMap.Intake.ACTUATOR2);
        pidControllerWinch = primary_winch.getPIDController();

        engaged = false;

        initialPosition = 0;
        //PID coefficients
        kP_winch = 0.0; //TODO: untested
        kI_winch = 0.0; // TODO: untested
        kD_winch = 0.0; //TODO: untested
        kMaxOutput = 1;
        kMinOutput = -1;
        kFF_winch = 0;
        kIz_winch = 0;
        maxRPM = 100;
        pidControllerWinch.setP(kP_winch);
        pidControllerWinch.setI(kI_winch);
        pidControllerWinch.setD(kD_winch);
        pidControllerWinch.setFF(kFF_winch);
        pidControllerWinch.setIZone(kIz_winch);
        pidControllerWinch.setOutputRange(kMinOutput, kMaxOutput);

    }
    /**
     * checks to see if an instance of climber exists, and if not then it makes one
     * @return an instance of climber
     */
    public static Climber getInstance() {
        if (instance == null) {
            return new Climber();
        }
        return instance;
    }
    /**
     * gets what the position on the arm is at
     * @return whatever value the arms position is
     */
    public double getArmPosition(){
        return arm.getEncoder().getPosition();
    }
    /**
     * gets what the encoder on the winch is getting
     * @return the winch position
     */
    public double getWinchPosition(){
        return primary_winch.getEncoder().getPosition();
    }
    /**
     * resets the winch position
     */
    public void resetWinchPosition(){
        primary_winch.getEncoder().setPosition(0);
        secondary_winch.getEncoder().setPosition(0);
    }
    /**
     * resets the arm position
     */
    public void resetArmPosition(){
        arm.getEncoder().setPosition(0);
    }

    /**
     * moves the arm-hook apparatus up to a given setpoint up.
     * @author dri
     */

    public void moveArmUp(double targetDist, double power) {
        double pGain = .1;
        double distanceToTarget = Math.abs(targetDist) - Math.abs(arm.getEncoder().getPosition() - initialPosition);
    
        double targetSpeed = pGain * (power * distanceToTarget);
    
        if (distanceToTarget > 0) {
            arm.set(targetSpeed); // TODO: code sanity check
        }
    }

    /**
     * moves the arm down to a setpoint of 0, or all the way down.
     */
    public void moveArmDown(){
        pidControllerArm.setReference(RobotMap.Climber.ARM_DOWNPOINT, ControlType.kPosition);
    }

    /**
     * moves the winch to a target setpoint that is made in RobotMap, might want to change if you want to 
     * make it a method that takes in a setpoint, however I dont think that was needed
     */
    public void activateWinch(){
      pidControllerWinch.setReference(RobotMap.Climber.WINCH_SETPOINT, ControlType.kPosition);
    }

    public boolean getEngaged() {
        return engaged;
    }

    public void setEngaged(boolean setEngaged) {
        engaged = setEngaged;
    }

    public void testWinch() {
        primary_winch.set(.1);
    }

    public void testWinchAndEngage() {
        if (!engaged) {
            primary_winch.set(.1);
        } else {
            primary_winch.set(0);
        }

    }

    /**
     * Raise the intake.
     * @author igc
     */
    public void engageBrake() {
        if (!engaged) {
            actuator.set(Value.kForward);
        }
    }

    /**
     * Lower the intake.
     * @author igc
     */
    public void disengageBrake() {
        actuator.set(Value.kReverse);
    }


}

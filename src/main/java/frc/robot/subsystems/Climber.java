package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.OI;
import frc.robot.RobotMap;

/**
 * This is the code for the climber. It initializes motor controllers/pneumatics
 * and has methods for various functions of the climber.
 * @author igc
 */
public class Climber {
    private static Climber instance;

    private CANSparkMax primary_winch, secondary_winch, arm;
    private CANPIDController pidControllerArm, pidControllerWinch;

    public double kP_arm, kP_winch, kI_arm, kI_winch, kD_arm, kD_winch,
        kMaxOutput, kMinOutput, maxRPM, kFF_arm, kFF_winch, kIz_arm, kIz_winch;

    private DigitalInput isDeployed;
    private double initialPosition;
    private DoubleSolenoid actuator;

    private Climber() {
        primary_winch = new CANSparkMax(RobotMap.Climber.PRIMARY_WINCH, MotorType.kBrushless);
        secondary_winch = new CANSparkMax(RobotMap.Climber.SECONDARY_WINCH, MotorType.kBrushless);
        primary_winch.setInverted(RobotMap.Climber.PRIMARY_WINCH_IS_INVERTED); //positive is winching
        secondary_winch.follow(primary_winch);
        
        arm = new CANSparkMax(RobotMap.Climber.ARM, MotorType.kBrushless);
        arm.setInverted(RobotMap.Climber.ARM_IS_INVERTED);

        actuator = new DoubleSolenoid(RobotMap.Climber.ACTUATOR1, RobotMap.Climber.ACTUATOR2);
        isDeployed = new DigitalInput(RobotMap.Climber.ARM_DEPLOY_SENSOR);

        pidControllerWinch = primary_winch.getPIDController();

        initialPosition = 0;
        // PID coefficients
        kP_winch = 0.0; // TODO: untested
        kI_winch = 0.0; // TODO: untested
        kD_winch = 0.0; // TODO: untested
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

        resetArmPosition();

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
     * retrieves the position of the arm
     * @return the arm position in ticks
     */
    public double getArmPosition(){
        return arm.getEncoder().getPosition();
    }

    /**
     * retrieves the position of the winch
     * @return the winch position in ticks
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
     * Moves the arm-hook apparatus up to a given setpoint up.
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
     * moves the winch to its target setpoint.
     */
    public void activateWinch(){
      pidControllerWinch.setReference(RobotMap.Climber.WINCH_SETPOINT, ControlType.kPosition);
    }

    /**
     * TODO: remove this.
     * runs the winch at a ludicrously slow speed for testing.
     */

    public void stopWinch() {
        primary_winch.set(0);
    }

    /**
     * Engage brake
     * @author igc
     */
    public void engageBrake() {
        actuator.set(Value.kForward);

    }

    /**
     * Disengage brake
     * @author igc
     */
    public void disengageBrake() {
        actuator.set(Value.kReverse);
    }

    public void manualClimb(double holdPower) {
        if(OI.driver.getLY() > holdPower) {
            disengageBrake();
            primary_winch.set(OI.driver.getLY());
        } if(OI.driver.getLY() < -.05) {
            disengageBrake();
            primary_winch.set(OI.driver.getLY() + holdPower);
        } else {
           engageBrake(); 
        }
    }

    public void manualArm(double speed) {
        arm.set(speed);
    }


    public void stopArm() {
        arm.set(0);
    }

    public void setSpeedWinch(double speed) {
        primary_winch.set(speed);
        
    }

    public boolean getArmDeploy() {
        return isDeployed.get();
    }
}

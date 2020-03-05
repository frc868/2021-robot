package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    private Climber() {
        primary_winch = new CANSparkMax(RobotMap.Climber.PRIMARY_WINCH, MotorType.kBrushless);
        secondary_winch = new CANSparkMax(RobotMap.Climber.SECONDARY_WINCH, MotorType.kBrushless);
        arm = new CANSparkMax(RobotMap.Climber.ARM, MotorType.kBrushless);
        secondary_winch.follow(primary_winch);
        pidControllerArm = arm.getPIDController();
        pidControllerWinch = primary_winch.getPIDController();

        initialPosition = 0;
        //PID coefficients
        kP_arm = 0.0; //TODO: untested
        kP_winch = 0.0; //TODO: untested
        kI_arm = 0.0; // TODO: untested
        kI_winch = 0.0; // TODO: untested
        kD_arm = 0.0; // TODO: untested
        kD_winch = 0.0; //TODO: untested
        kMaxOutput = 1;
        kMinOutput = -1;
        kFF_arm = 0;
        kFF_winch = 0;
        kIz_arm = 0;
        kIz_winch = 0;
        maxRPM = 100;
        pidControllerArm.setP(kP_arm);
        pidControllerArm.setI(kI_arm);
        pidControllerArm.setD(kD_arm);
        pidControllerArm.setOutputRange(kMinOutput, kMaxOutput);
        pidControllerArm.setIZone(kIz_arm);
        pidControllerArm.setFF(kFF_arm);
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
     * gets what the postion on the arm is at
     * @return whatever value the arms position is
     */
    public double getArmPostion(){
        return arm.getEncoder().getPosition();
    }
    /**
     * gets what the encoder on the winch is getting
     * @return the winch postion
     */
    public double getWinchPostion(){
        return primary_winch.getEncoder().getPosition();
    }
    /**
     * resets the winch postion
     */
    public void resetWinchPostion(){
        primary_winch.getEncoder().setPosition(0);
        secondary_winch.getEncoder().setPosition(0);
    }
    /**
     * resets the arm postion
     */
    public void resetArmPostion(){
        arm.getEncoder().setPosition(0);
    }
    /**
     * moves the arm-hook apparatus up to a given setpoint up.
     */
    public void moveArmUp(){ 
        pidControllerArm.setReference(RobotMap.Climber.ARM_SETPOINT, ControlType.kPosition);
    }

    public void toSetpoint(double targetDist, double power) {
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
}
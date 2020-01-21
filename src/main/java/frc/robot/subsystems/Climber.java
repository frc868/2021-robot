package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import  edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
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
    private SpeedControllerGroup winchGroup;
    private PIDController pid_arm, pid_winch;
    private static final double kP_arm = 0, kI_arm = 0, kD_arm = 0, kP_winch = 0, kI_winch = 0, kD_winch = 0;

    private Climber() {
        primary_winch = new CANSparkMax(RobotMap.Climber.PRIMARY_WINCH, MotorType.kBrushless);
        secondary_winch = new CANSparkMax(RobotMap.Climber.SECONDARY_WINCH, MotorType.kBrushless);
        arm = new CANSparkMax(RobotMap.Climber.ARM, MotorType.kBrushless);
        winchGroup = new SpeedControllerGroup(primary_winch, secondary_winch);
        pid_arm = new PIDController(kP_arm, kI_arm, kD_arm);
        pid_winch = new PIDController(kP_winch, kI_winch, kD_winch);
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
        double currentPostition = getArmPostion();
        if(RobotMap.Climber.ARM_SETPOINT != currentPostition){
            currentPostition = getArmPostion();
            double speed = pid_arm.calculate(currentPostition, RobotMap.Climber.ARM_SETPOINT);
            arm.set(speed);
        }
    }
    /**
     * moves the arm down to a setpoint of 0, or all the way down.
     */
    public void moveArmDown(){
        double currentPostition = getArmPostion();
        if(currentPostition != currentPostition){
            currentPostition = getArmPostion();
            double speed = pid_arm.calculate(currentPostition, RobotMap.Climber.ARM_DOWNPOINT);
            arm.set(speed);
        }
    }
    /**
     * moves the winch to a target setpoint that is made in RobotMap, might want to change if you want to 
     * make it a method that takes in a setpoint, however I dont think that was needed
     */
    public void activateWinch(){
        double currentPostition = getWinchPostion();
        if(RobotMap.Climber.WINCH_SETPOINT != currentPostition){
            currentPostition = getWinchPostion();
            double speed = pid_winch.calculate(currentPostition, RobotMap.Climber.WINCH_SETPOINT);
            winchGroup.set(speed);
        }   
    }
}
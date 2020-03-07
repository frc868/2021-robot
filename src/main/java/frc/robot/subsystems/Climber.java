package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
    private CANPIDController pidController;

    // winch PID constants
    private double kP, kI, kD, kFF, kIa;
    // global PID constants
    private double kMaxOutput, kMinOutput;

    private DigitalInput isDeployed;
    private double initialPosition;
    private boolean lastArmState;
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

        pidController = primary_winch.getPIDController();

        initialPosition = 0;
        // PID coefficients
        kP = 0.0; // TODO: untested
        kI = 0.0; // TODO: untested
        kD = 0.0; // TODO: untested
        kMaxOutput = 1;
        kMinOutput = -1;
        kFF = 0;
        kIa = 0;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kFF);
        pidController.setIMaxAccum(kIa, 0);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        if (this.kI == 0) {
            pidController.setIAccum(0);
        }

        resetArmPosition();
        resetWinchPosition();
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
    public void resetArmPosition() {
        arm.getEncoder().setPosition(0);
    }

    /**
     * Moves the arm-hook apparatus up to a given setpoint up.
     * @author dri
     */
    public void moveArmUp(double power) {
        if (getArmDeployToggled()) {
            arm.set(0);
        } else {
            arm.set(power);
        }
    }

    public void moveArmDown(double power) {
        if (getArmPosition() <= 1) {
            arm.set(0);
        } else {
            arm.set(power);
        }
    }

    /**
     * moves the winch to its target setpoint.
     */
    public void activateWinch() {
        pidController.setReference(RobotMap.Climber.WINCH_SETPOINT, ControlType.kPosition);
    }

    /**
     * Kills the winch motor.
     */
    public void stopWinch() {
        primary_winch.set(0);
    }

    /**
     * Engages the brake.
     * @author igc
     */
    public void engageBrake() {
        actuator.set(Value.kForward);

    }

    /**
     * Disengages the brake.
     * @author igc
     */
    public void disengageBrake() {
        actuator.set(Value.kReverse);
    }

    /**
     * Climbs based on joystick inputs with no closed-loop control.
     * @param holdPower the power to hold the motor at
     */
    public void manualClimb(double power) {
        if (OI.operator.getLY() > .5) {
            disengageBrake();
            primary_winch.set(power);
        } else if (OI.driver.getLY() < -.5) {
            disengageBrake();
            primary_winch.set(-power);
        } else {
           engageBrake(); 
           stopWinch();
        }
    }

    public void manualArm(double armPower) {
        if (OI.operator.getRY() > 0) {
            setSpeedArm(armPower);
        } else if (OI.operator.getLY() < 0) {
            setSpeedArm(-armPower);
        } else {
           setSpeedArm(0);
        }
    }

    public void setSpeedArm(double speed) {
        if (getArmDeployToggled() || (getArmPosition() == 0)) {
            arm.set(0);
        } else {
            arm.set(speed);
        }
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

    private boolean getArmDeployToggled() {
        if (getArmDeploy() != lastArmState) {
            lastArmState = getArmDeploy();
            if (lastArmState == RobotMap.Climber.ARM_LAST_STATE_VALUE) {
                return true;
            }
        }
        return false;
    }
}

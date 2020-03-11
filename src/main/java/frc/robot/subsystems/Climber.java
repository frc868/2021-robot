package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;
import frc.robot.helpers.Helper;

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

    private double kMaxOutput, kMinOutput;

    private DigitalInput isDeployed;
    private double initialPosition;
    private boolean lastArmState;
    private DoubleSolenoid actuator;
    private DoubleSolenoid hook_deploy;

    private Climber() {
        primary_winch = new CANSparkMax(RobotMap.Climber.PRIMARY_WINCH, MotorType.kBrushless);
        secondary_winch = new CANSparkMax(RobotMap.Climber.SECONDARY_WINCH, MotorType.kBrushless);
        primary_winch.setInverted(RobotMap.Climber.PRIMARY_WINCH_IS_INVERTED); //positive is winching
        secondary_winch.follow(primary_winch);
        
        arm = new CANSparkMax(RobotMap.Climber.ARM, MotorType.kBrushless);
        arm.setInverted(RobotMap.Climber.ARM_IS_INVERTED);
        arm.setIdleMode(IdleMode.kCoast);

        actuator = new DoubleSolenoid(RobotMap.Climber.ACTUATOR1, RobotMap.Climber.ACTUATOR2);
        hook_deploy = new DoubleSolenoid(RobotMap.Climber.HOOK_ACTUATOR, RobotMap.Climber.HOOK_ACTUATOR_UNUSED);
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

        // resetArmPosition();
        resetWinchPosition();
    }

    /**
     * Checks to see if an instance of climber exists, and if not then it makes one.
     * @return an instance of climber
     */
    public static Climber getInstance() {
        if (instance == null) {
            return new Climber();
        }
        return instance;
    }

    /**
     * Retrieves the position of the arm motor.
     * @return the arm position in ticks
     */
    public double getArmPosition(){
        return arm.getEncoder().getPosition();
    }

    /**
     * Retrieves the position of the winch motors.
     * @return the winch position in ticks
     */
    public double getWinchPosition(){
        return primary_winch.getEncoder().getPosition();
    }

    /**
     * Resets the winch position.
     */
    public void resetWinchPosition(){
        primary_winch.getEncoder().setPosition(0);
        secondary_winch.getEncoder().setPosition(0);
    }

    /**
     * Resets the arm position.
     */
    public void resetArmPosition() {
        arm.getEncoder().setPosition(0);
    }

    /**
     * Moves the winch to its target setpoint.
     */
    public void automaticWinch() {
        pidController.setReference(RobotMap.Climber.WINCH_SETPOINT, ControlType.kPosition);
    }

    /**
     * Kills the winch motor.
     */
    public void stopWinch() {
        primary_winch.set(0);
    }

    /**
     * Releases the hook on the arm to go to its maximum height.
     */
    public void deployHook() {
        arm.setIdleMode(IdleMode.kCoast);
        hook_deploy.set(Value.kForward);
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
     * Sets the arm motor to a given speed. Note: this motor should only be run at a negative
     * speed, since the arm's hook is pneumatically deployed upwards.
     * @param speed the speed to run the arm at
     */
    public void setSpeedArm(double speed) {
        arm.setIdleMode(IdleMode.kBrake); // brake mode to slow descent
        arm.set(speed);
    }

    /**
     * Stops the arm
     */
    public void stopArm() {
        arm.set(0);
    }

    /**
     * Sets the speed of the winch. Positive speeds correspond to winching up, and negative speeds
     * correspond to lowering.
     * @param speed the speed to run winch motors at
     */
    public void setSpeedWinch(double speed) {
        primary_winch.set(speed);
        
    }

    /**
     * Get whether the arm has been deployed.
     * @return deployment state of the arm
     */
    public boolean getArmDeploy() {
        return isDeployed.get();
    }
}

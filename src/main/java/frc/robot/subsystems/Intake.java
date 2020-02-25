package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.helpers.Helper;

/**
 * Intakes power cells with a singular NEO 550.
 * @author ai
 */ 
public class Intake {
    private static Intake instance;
    private CANSparkMax motor;
    private DoubleSolenoid actuator;

    private Timer timer;
    private boolean timerStarted = false;

    private Intake() {
        motor = new CANSparkMax(RobotMap.Intake.MOTOR, MotorType.kBrushless);
        actuator = new DoubleSolenoid(RobotMap.Intake.ACTUATOR1, RobotMap.Intake.ACTUATOR2);
        timer = new Timer();

        motor.setInverted(RobotMap.Intake.MOTOR_IS_INVERTED);
    }

    /**
     * Returns a singleton instance of the intake subsystem.
     */
    public static Intake getInstance() {
        if (instance == null) {
            return new Intake();
        }

        return instance;
    }
    
    /**
     * Sets the speed for the motor motor (secondary motor follows)
     * @param speed the speed to set from -1 to 1
     */
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    /** 
     * Retrieves the speed for the motor motor (and consequently secondary)
     * @return the speed, from -1 to 1
     */
    public double getIntakeSpeed() {
        return motor.get();
    }

    /**
     * sets the actuator to the position it is not currently in
     * @author acr
     */
    public void toggle() {
        if (actuator.get() == Value.kForward) {
            actuator.set(Value.kReverse);
        } else if (actuator.get() == Value.kReverse) {
            actuator.set(Value.kForward);
        }
    }

    /**
     * Raise the intake.
     * @author acr
     */
    public void actuatorUp() {
        actuator.set(Value.kReverse);
    }

    /**
     * Lower the intake.
     * @author acr
     */
    public void actuatorDown() {
        actuator.set(Value.kForward);
    }

    /**
     * intakes balls until either the hopper is full or a time delay is exceeded.
     * this sanity checking is to ensure we don't waste time intaking <i>forever</i>
     * in autonomous.
     * @param delay the threshold for how long the intake can run, in seconds
     * @param power the power to run the intake at from -1 to 1
     * @return false if not finished, true if finished
     * @author hrl
     */
    public boolean intakeUntilFull(double delay, double power) {
        if (!timerStarted) {
            timer.reset();
            timer.start();
            this.timerStarted = true;
        }

        // while we're either not full or we're behind the delay...
        if ((Robot.hopper.getBallCount() < 5) || (timer.get() < delay)) {
            this.setSpeed(power);
            return false;
        } else { // either delay elapsed or we're full
            timer.stop();
            this.timerStarted = false;
            return true;
        }
    }
}

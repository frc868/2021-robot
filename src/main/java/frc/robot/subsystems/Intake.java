package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

    private Intake() {
        motor = new CANSparkMax(RobotMap.Intake.MOTOR, MotorType.kBrushless);
        actuator = new DoubleSolenoid(RobotMap.Intake.ACTUATOR1, RobotMap.Intake.ACTUATOR2);

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
        actuator.set(Value.kForward);
    }

    /**
     * Lower the intake.
     * @author acr
     */
    public void actuatorDown() {
        actuator.set(Value.kReverse);
    }
}

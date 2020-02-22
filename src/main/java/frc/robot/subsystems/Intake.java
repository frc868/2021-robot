package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotMap;
import frc.robot.helpers.Helper;

/**
 * Intakes power cells with a singular NEO 550.
 * @author ai
 */ 
public class Intake {
    private static Intake instance;
    private CANSparkMax motor;

    private Intake() {
        motor = new CANSparkMax(RobotMap.Intake.MOTOR, MotorType.kBrushless);

        motor.setInverted(RobotMap.Intake.MOTOR_IS_INVERTED);
    }

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
        motor.set(Helper.boundValue(speed, -1, 1));
    }

    /** 
     * Retrieves the speed for the motor motor (and consequently secondary)
     * @return the speed, from -1 to 1
     */
    public double getIntakeSpeed() {
        return motor.get();
    }
}

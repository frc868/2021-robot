package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;
import frc.robot.helpers.Helper;

/**
 * This is the code for the power cell intake. It initiallizes two
 * Talon motors and makes the primary follow the secondary motor. 
 * 
 * @author ai
 */ 
public class Intake {
    private static Intake instance;
    private WPI_TalonSRX primary;
    private WPI_TalonSRX secondary;

    private Intake() {
        primary = new WPI_TalonSRX(RobotMap.Intake.PRIMARY);
        secondary = new WPI_TalonSRX(RobotMap.Intake.SECONDARY);

        secondary.follow(primary);
    }

    public static Intake getInstance() {
        if (instance == null) {
            return new Intake();
        }

        return instance;
    }
    
    /**
     * Sets the speed for the primary motor (secondary motor follows) 
     * @param speed the speed to set from -1 to 1
     */
    public void setSpeed(double speed) {
        primary.set(Helper.boundValue(speed, -1, 1));
    }

    /** 
     * Retrieves the speed for the primary motor (and consequently secondary)
     * @return the speed, from -1 to 1
     */
    public double getIntakeSpeed() {
        return primary.get();
    }
}
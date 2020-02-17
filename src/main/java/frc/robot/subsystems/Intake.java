package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
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

    private Timer timer;
    private boolean timerStarted = false;

    private Intake() {
        primary = new WPI_TalonSRX(RobotMap.Intake.PRIMARY);
        secondary = new WPI_TalonSRX(RobotMap.Intake.SECONDARY);

        timer = new Timer();

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

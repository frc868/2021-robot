package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

/**
 * The Hopper subsystem consists of 3 motors to move the power cells in stages.
 * The bottom of the hopper is not indexed, and can hold three balls. They then
 * transition into the tower, where they are indexed using 3 individual sensors.
 * 
 * @author dri, igc
 */
public class Hopper {
    private static Hopper instance;

    private DigitalInput botSensorL;
    private DigitalInput botSensorR;
    private DigitalInput midLeftLim;
    private DigitalInput topLeftLim;
    private DigitalInput beamBreak;

    private WPI_TalonSRX floor;
    private WPI_TalonSRX upToShooter;
    private WPI_TalonSRX blueWheels;

    // a state variable to control the number of balls currently in the hopper
    private int count = 3;

    // store the last value of the limit switches to see if they have been triggered
    // after
    private boolean lastBotState;
    private boolean lastMidState;
    private boolean lastTopState;

    private boolean isCompBot = true;

    private Hopper(boolean compBot) {
        // botSensorL = new DigitalInput(RobotMap.Hopper.Sensors.BOT_SENSOR_PORT_LEFT);
        // botSensorR = new DigitalInput(RobotMap.Hopper.Sensors.BOT_SENSOR_PORT_RIGHT);
        
        // midLeftLim = new DigitalInput(RobotMap.Hopper.Sensors.MID_LEFT);
        // topLeftLim = new DigitalInput(RobotMap.Hopper.Sensors.TOP_LEFT);

        beamBreak = new DigitalInput(RobotMap.Hopper.Sensors.HOPPER_BEAM_BREAK);

        floor = new WPI_TalonSRX(RobotMap.Hopper.Motor.HOPPER_FLOOR);
        upToShooter = new WPI_TalonSRX(RobotMap.Hopper.Motor.UP_TO_SHOOTER);
        blueWheels = new WPI_TalonSRX(RobotMap.Hopper.Motor.BLUE_WHEELS);
        floor.setInverted(RobotMap.Hopper.Motor.HOPPER_FLOOR_IS_INVERTED);
        upToShooter.setInverted(RobotMap.Hopper.Motor.UP_TO_SHOOTER_IS_INVERTED);

        upToShooter.setNeutralMode(NeutralMode.Brake);

        // lastBotState = getBotSensor();
        // lastMidState = getMidLimit();
        // lastTopState = getTopLimit();

        this.isCompBot = compBot;
    }

    /**
     * Returns a singular instance of the Intake subsystem.
     */
    public static Hopper getInstance(boolean compBot) {
        if (instance == null) {
            instance = new Hopper(compBot);
        }
        return instance;
    }

    // ============ ACTUATION ============

    /**
     * Stops the floor and upToShooter motors.
     */
    public void stop() {
        floor.set(0);
        upToShooter.set(0);
        blueWheels.set(0);
    }

    /**
     * Indexes hopper.
     * 
     * @author igc
     */
    public void update() {
        // count();
        // if (isCompBot) {
            while(!beamBreak.get()) {
                // System.out.println(beamBreak.get());
                blueWheels.set(0);
                upToShooter.set(1);
                floor.set(0);
            }

            floor.set(-0.6);
            upToShooter.set(1);
            blueWheels.set(-0.6);
            // floor.set(RobotMap.Hopper.Speeds.CompBot.Update.FLOOR_SPEED);
            // upToShooter.set(RobotMap.Hopper.Speeds.CompBot.Update.UP_TO_SHOOTER_SPEED);
            // blueWheels.set(RobotMap.Hopper.Speeds.CompBot.Update.BLUE_SPEED);
        // } else {
        //     floor.set(RobotMap.Hopper.Speeds.PracticeBot.Update.FLOOR_SPEED);
        //     upToShooter.set(RobotMap.Hopper.Speeds.PracticeBot.Update.UP_TO_SHOOTER_SPEED);
        //     blueWheels.set(RobotMap.Hopper.Speeds.PracticeBot.Update.BLUE_SPEED);
        // }
    }

    /**
     * called when the driver is ready to shoot (pushing the button on the
     * controller) sets the floor speed to the tested value necessary to feed
     */
    public void reverse(double speedOthers, double speedFloor) {
        floor.set(-speedFloor);
        upToShooter.set(1);
        blueWheels.set(-speedOthers);
    }

    /**
     * Called when the driver is ready to shoot. Doesn't index balls until
     * shooterAtSpeed argument is true, meaning the shooter has spun up.
     * @param shooterAtSpeed whether the shooter is at target RPM
     */
    public void forward(boolean atTarget) {
        if (atTarget) {
            floor.set(RobotMap.Hopper.Speeds.Forward.FLOOR_SPEED);
            upToShooter.set(RobotMap.Hopper.Speeds.Forward.UP_TO_SHOOTER_SPEED);
            blueWheels.set(RobotMap.Hopper.Speeds.Forward.BLUE_SPEED);
            upToShooter.set(1);
        } else {
            this.stop();
        }
    }

    /**
     * An experimental shooting method which ignores top sensor logic as an attempt to increase
     * indexing speed. TODO: needs testing.
     * @param atTarget whether the shooter is at its target
     */
    // public void forwardShot(boolean atTarget) {
    //     if (atTarget) {
    //         if (getMidLimitToggled() || !getMidLimit()) {
    //             floor.set(RobotMap.Hopper.Speeds.Forward.FLOOR_SPEED);
    //             upToShooter.set(RobotMap.Hopper.Speeds.Forward.UP_TO_SHOOTER_SPEED);
    //             blueWheels.set(RobotMap.Hopper.Speeds.Forward.BLUE_SPEED);
    //         } else {
    //             upToShooter.set(1);
    //         }
    //     } else {
    //         this.stop();
    //     }
    // }

    /**
     * Runs the hopper without logic. <i>Avoid using this.</i>
     */
    public void shootNoLogic() {
        if (isCompBot) {
            floor.set(RobotMap.Hopper.Speeds.CompBot.Update.FLOOR_SPEED);
            upToShooter.set(RobotMap.Hopper.Speeds.CompBot.Update.UP_TO_SHOOTER_SPEED);
            blueWheels.set(RobotMap.Hopper.Speeds.CompBot.Update.BLUE_SPEED);
        } else {
            floor.set(0);
            upToShooter.set(0);
            blueWheels.set(0);
        }
    }

    // ========================================================

    // ============ UTILITIES ============

    // /**
    //  * Determine whether we are in a state where shooting is possible.
    //  */
    // public boolean readyToShoot() {
    //     return getTopLimit();
    // }

    // /**
    //  * Increments count variable for number of balls stored in the hopper.
    //  */
    // private void count() {
    //     if (getTopLimitToggled()) {
    //         count--;
    //     }

    //     if (getMidLimitToggled()) {
    //         count++;
    //     }
    // }

    /**
     * returns count of how many balls are currently held in the hopper     * 
     * @return count
     */
    public int getBallCount() {
        return count;
    }

    // ========================================================


    // ============ SENSORS ============

    /**
    //  * Returns the state of the top limits.
    //  */
    // public boolean getTopLimit() {
    //     return !topLeftLim.get();
    // }

    // /**
    //  * Returns the state of the mid limit.
    //  */
    // public boolean getMidLimit() {
    //     return !midLeftLim.get();
    // }

    // /**
    //  * Returns true if one of the bottom sensors is toggled.
    //  * @return value of bottom sensors
    //  */
    // public boolean getBotSensor() {
    //     return getBotLeftSensor() || getBotRightSensor();
    // }

    // /**
    //  * Returns true if the bottom left sensor is toggled.
    //  * @return value 
    //  */
    // public boolean getBotLeftSensor() {
    //     return !botSensorL.get();
    // }

    // public boolean getBotRightSensor() {
    //     return !botSensorR.get();
    // }

    // /**
    //  * returns true if bottom limit switches are toggled from true to false
    //  * (unsimplified expression: current left state is false and last state is true,
    //  * or current right state is false and last state is true)
    //  * 
    //  * @return toggled
    //  */
    // private boolean getMidLimitToggled() {
    //     if (getMidLimit() != lastMidState) {
    //         lastMidState = getMidLimit();
    //         if (lastMidState == RobotMap.Hopper.Sensors.MID_LAST_STATE_VALUE) {
    //             return true;
    //         }
    //     }
    //     return false;

    // }

    // /**
    //  * returns true if top limit switches are toggled from true to false
    //  * (unsimplified expression: current left state is false and last state is true,
    //  * or current right state is false and last state is true)
    //  * 
    //  * @return toggled
    //  */
    // private boolean getTopLimitToggled() {
    //     if (getTopLimit() != lastBotState) {
    //         lastTopState = getTopLimit();
    //         if (lastTopState == RobotMap.Hopper.Sensors.TOP_LAST_STATE_VALUE) {
    //             return true;
    //         }
    //     }
    //     return false;
    // }

    @Override
    public String toString() {
        return "Count: " + count + " , bottom: ";
    }
    // ========================================================
}

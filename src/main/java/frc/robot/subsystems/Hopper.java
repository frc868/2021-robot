package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotMap;

/**
 * The Hopper subsystem consists of 3 motors to move the power cells in stages.
 * Positions one and two are on the bottom, position three is the transition to
 * the upper level of the hopper, and positions four and five are on the top
 * level.
 * 
 * @author dri
 */

public class Hopper {
    private static Hopper instance;

    private DigitalInput botSensor;
    private DigitalInput midLeftLim;
    private DigitalInput topLeftLim;

    private WPI_TalonSRX belt;
    private WPI_TalonSRX feeder;
    private WPI_TalonSRX blueWheels;

    // a state variable to control the number of balls currently in the hopper
    private int count = 3;
    // a state variable to control whether the driver has overriden the autonomous
    // functions
    private boolean driverOverride;

    private double initialBeltPosition;
    private double initialFeederPosition;

    // store the last value of the limit switches to see if they have been triggered
    // after
    private boolean lastBotState;
    private boolean lastMidState;
    private boolean lastTopState;

    private Hopper() {
        driverOverride = false;

        botSensor = new DigitalInput(RobotMap.Hopper.Sensors.BOT_SENSOR_PORT);
        midLeftLim = new DigitalInput(RobotMap.Hopper.Sensors.MID_LEFT);
        topLeftLim = new DigitalInput(RobotMap.Hopper.Sensors.TOP_LEFT);

        belt = new WPI_TalonSRX(RobotMap.Hopper.Motor.HOPPER_FLOOR);
        feeder = new WPI_TalonSRX(RobotMap.Hopper.Motor.FEEDER);
        blueWheels = new WPI_TalonSRX(RobotMap.Hopper.Motor.BLUE_WHEELS);
        belt.setInverted(RobotMap.Hopper.Motor.HOPPER_FLOOR_IS_INVERTED);
        feeder.setInverted(RobotMap.Hopper.Motor.FEEDER_IS_INVERTED);

        feeder.setNeutralMode(NeutralMode.Brake);

        lastBotState = getBotSensor();
        lastMidState = getMidLimit();
        lastTopState = getTopLimit();

        initialBeltPosition = belt.getSensorCollection().getQuadraturePosition();
        initialFeederPosition = feeder.getSensorCollection().getQuadraturePosition();
    }

    /**
     * Returns a singular instance of the Intake subsystem.
     */
    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    // ============ ACTUATION ============

    /**
     * Stops the belt and feeder motors. Untested.
     */
    public void stop() {
        belt.set(0);
        feeder.set(0);
        blueWheels.set(0);
    }

    /**
     * Indexes hopper.
     * 
     * @author igc
     */
    public void update() {
        count();
        if (!getTopLimit() && (!getMidLimit() || getBotSensor())) {
            belt.set(RobotMap.Hopper.Speeds.Update.BELT_SPEED);
            feeder.set(RobotMap.Hopper.Speeds.Update.FEEDER_SPEED);
            blueWheels.set(RobotMap.Hopper.Speeds.Update.BLUE_SPEED);
        } else {
            stop();
        }
    }

    /**
     * called when the driver is ready to shoot (pushing the button on the
     * controller) sets the belt speed to the tested value necessary to feed
     */
    public void reverse(double speed) {
        driverOverride = true;
        belt.set(-speed);
        feeder.set(-speed);
        blueWheels.set(-speed);
    }

    /**
     * called when the driver is ready to shoot (pushing the button on the
     * controller) sets the belt speed to the tested value necessary to feed
     */
    public void forward() {
        driverOverride = true;
        if (getMidLimitToggled() || (!getTopLimit() && !getMidLimit())) {
            belt.set(RobotMap.Hopper.Speeds.Forward.BELT_SPEED);
            feeder.set(RobotMap.Hopper.Speeds.Forward.FEEDER_SPEED);
            blueWheels.set(RobotMap.Hopper.Speeds.Forward.BLUE_SPEED);
        } else {
            feeder.set(1);
        }
    }

    // ========================================================

    // ============ UTILITIES ============

    /**
     * Determine whether we are in a state where shooting is possible.
     */
    public boolean readyToShoot() {
        return getTopLimit();
    }

    /**
     * Increments count variable for number of balls stored in the hopper.
     */
    private void count() {
        if (getTopLimitToggled()) {
            count--;
        }

        if (getMidLimitToggled()) {
            count++;
        }
    }

    /**
     * resets the driver override trigger
     * 
     * @author hrl
     */
    public void resetOverride() {
        driverOverride = false;
    }

    /**
     * returns count of how many balls are currently held in the hopper     * 
     * @return count
     */
    public int getBallCount() {
        return count;
    }

    @Override
    public String toString() {
        return "Count: " + count + " , bottom: " + getBotSensor();
    }
    // ========================================================


    // ============ SENSORS ============

    /**
     * Returns the state of the top limits.
     */
    public boolean getTopLimit() {
        return !topLeftLim.get();
    }

    /**
     * Returns the state of the mid limits.
     */
    public boolean getMidLimit() {
        return !midLeftLim.get();
    }

    /**
     * Returns true if beam break senses
     * 
     * @author igc
     */
    public boolean getBotSensor() {
        return !botSensor.get();
    }

    /**
     * returns true if bottom limit switches are toggled from true to false
     * (unsimplified expression: current left state is false and last state is true,
     * or current right state is false and last state is true)
     * 
     * @return toggled
     */
    private boolean getMidLimitToggled() {
        if (getMidLimit() != lastMidState) {
            lastMidState = getMidLimit();
            if (lastMidState == RobotMap.Hopper.Sensors.MID_LAST_STATE_VALUE) {
                return true;
            }
        }
        return false;

    }

    /**
     * returns true if top limit switches are toggled from true to false
     * (unsimplified expression: current left state is false and last state is true,
     * or current right state is false and last state is true)
     * 
     * @return toggled
     */
    private boolean getTopLimitToggled() {
        if (getTopLimit() != lastBotState) {
            lastTopState = getTopLimit();
            if (lastTopState == RobotMap.Hopper.Sensors.TOP_LAST_STATE_VALUE) {
                return true;
            }
        }
        return false;
    }
    // ========================================================
}

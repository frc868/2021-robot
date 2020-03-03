package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotMap;

/**
 * The Hopper subsystem consists of 3 motors to move the power cells in stages. Positions one and
 * two are on the bottom, position three is the transition to the upper level of the hopper, and
 * positions four and five are on the top level.
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
    // a state variable to control whether the driver has overriden the autonomous functions
    private boolean driverOverride;

    private double initialBeltPosition;
    private double initialFeederPosition;

    // store the last value of the limit switches to see if they have been triggered after
    private boolean lastBotState;
    private boolean lastMidState;
    private boolean lastTopState;

    private Hopper() {
        driverOverride = false;

        botSensor = new DigitalInput(RobotMap.Hopper.Sensors.BOT_SENSOR_PORT);
        midLeftLim = new DigitalInput(RobotMap.Hopper.Sensors.MID_LEFT);
        topLeftLim = new DigitalInput(RobotMap.Hopper.Sensors.TOP_LEFT);
        //topRightLim = new DigitalInput(RobotMap.Hopper.Sensors.TOP_RIGHT);

        belt = new WPI_TalonSRX(RobotMap.Hopper.Motor.HOPPER_FLOOR);
        feeder = new WPI_TalonSRX(RobotMap.Hopper.Motor.FEEDER);
        blueWheels = new WPI_TalonSRX(RobotMap.Hopper.Motor.BLUE_WHEELS);
        belt.setInverted(RobotMap.Hopper.Motor.HOPPER_FLOOR_IS_INVERTED);
        feeder.setInverted(RobotMap.Hopper.Motor.FEEDER_IS_INVERTED);

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

    /**
     * Stops the belt and feeder motors. Untested.
     */
    public void stop() {
        driverOverride = false;

        belt.set(0);
        feeder.set(0);
        blueWheels.set(0);
    }

    /**
     * Indexes hopper. 
     * @author igc
     */
    public void update(double value) {
        if(value > 0) {
            count();
            if (!getTopLimit() && (!getMidLimit() || getBotSensor())) {
                belt.set(0.5);
                feeder.set(0.6); //.8
                blueWheels.set(0.6);

            } else {
                stop();
            }
        }
    }

    /**
     * Determine whether we are in a state where shooting is possible.
     */
    public boolean readyToShoot() {
        return getTopLimit();
    }

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
     * @author igc
     */
    public boolean getBotSensor() {
        return !botSensor.get();
    }

    /**
     * returns true if bottom limit switches are toggled from true to false
     * (unsimplified expression:
     * current left state is false and last state is true, or current right state is false
     * and last state is true)
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
     * (unsimplified expression:
     * current left state is false and last state is true, or current right state is false
     * and last state is true)
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

    /**
     * Updates the current ball count in the hopper.
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
     * index balls in the hopper, but not the belts that feed into the shooter
     */
    private void cycleIntake() {
        double currentBeltPosition = belt.getSensorCollection().getQuadraturePosition();
        double currentFeederPosition = feeder.getSensorCollection().getQuadraturePosition();
        if (currentBeltPosition - initialBeltPosition < RobotMap.Hopper.ENC_COUNT_PER_CYCLE) {
            belt.set(RobotMap.Hopper.HOPPER_FLOOR_SPEED); // TODO: check motor speed with balls
        } else {
            belt.set(0);
        }

        if (currentFeederPosition - initialFeederPosition < RobotMap.Hopper.ENC_COUNT_PER_CYCLE) {
            feeder.set(RobotMap.Hopper.FEEDER_SPEED); // TODO: check motor speed with balls
        } else {
            feeder.set(0);
        }
    }

    /**
     * called when the driver is ready to shoot (pushing the button on the
     * controller) sets the belt speed to the tested value necessary to feed
     */
    public void shoot() {
        driverOverride = true;

        this.count();
        belt.set(RobotMap.Hopper.HOPPER_FLOOR_SPEED);
        feeder.set(RobotMap.Hopper.FEEDER_SPEED);
        blueWheels.set(RobotMap.Hopper.BLUE_SPEED);
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
        if(getMidLimitToggled() || (!getTopLimit() && !getMidLimit())) {
        belt.set(.6);
        feeder.set(1);
        blueWheels.set(.7);
        } else {
            feeder.set(1);  
        }
    }


    /**
     * resets the driver override trigger
     * @author hrl
     */
    public void resetOverride() {
        driverOverride = false;
    }

    /**
     * returns count of how many balls are currently held in the hopper
     * @return count
     */
    public int getBallCount() {
        return count;
    }

    @Override
    public String toString() {
        return "Count: " + count + " , bottom: " + getBotSensor();
    }
}

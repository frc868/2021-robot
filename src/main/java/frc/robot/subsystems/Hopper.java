package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
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

    private DigitalInput botLeftLim;
    private DigitalInput botRightLim;

    private DigitalInput topLeftLim;
    private DigitalInput topRightLim;

    private WPI_TalonSRX belt;
    private WPI_TalonSRX indexer;

    // a state variable to control the number of balls currently in the hopper
    private int count = 3;
    // a state variable to control whether the driver has overriden the autonomous functions
    private boolean driverOverride = false;

    private double initialBeltPosition;
    private double initialIndexerPosition;

    // store the last value of the limit switches to see if they have been triggered after
    private boolean lastBotState;
    private boolean lastTopState;

    private Hopper() {
        driverOverride = false;

        botLeftLim = new DigitalInput(RobotMap.Hopper.Limit.BOTTOM_LEFT);
        botRightLim = new DigitalInput(RobotMap.Hopper.Limit.BOTTOM_RIGHT);
        topLeftLim = new DigitalInput(RobotMap.Hopper.Limit.TOP_LEFT);
        topRightLim = new DigitalInput(RobotMap.Hopper.Limit.TOP_RIGHT);

        belt = new WPI_TalonSRX(RobotMap.Hopper.Motor.BELT);
        indexer = new WPI_TalonSRX(RobotMap.Hopper.Motor.INDEXER);
        belt.setInverted(RobotMap.Hopper.Motor.BELT_IS_INVERTED);
        indexer.setInverted(RobotMap.Hopper.Motor.INDEXER_IS_INVERTED);

        lastBotState = getBotLimit();
        lastTopState = getTopLimit();

        initialBeltPosition = belt.getSensorCollection().getQuadraturePosition();
        initialIndexerPosition = indexer.getSensorCollection().getQuadraturePosition();
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
     * Stops the belt and indexer motors. Untested.
     */
    private void stop() {
        belt.set(0);
        indexer.set(0);
    }

    /**
     * Updates the current state of the turret. To be called in robotPeriodic().
     */
    public void update() {
        if (driverOverride) {
            if (getBotLimitToggled()) {
                count++;
                cycleIntake();
            }
            else if (getTopLimitToggled()) {
                count--;
            }
            else if (getTopLimitToggled() && count >= 5) {
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
    private boolean getTopLimit() {
        return topLeftLim.get() || topRightLim.get();
    }

    /**
     * Returns the state of the bottom limits.
     */
    private boolean getBotLimit() {
        return botLeftLim.get() || botRightLim.get();
    }

    /**
     * returns true if bottom limit switch is toggled from true to false
     * (unsimplified expression:
     * current left state is false and last state is true, or current right state is false
     * and last state is true)
     * 
     * @return toggled
     */
    private boolean getBotLimitToggled() {
        return !(botLeftLim.get() && botRightLim.get()) && lastBotState;
    }

    /**
     * Gets the current state of the top limit switches.
     */
    private boolean getTopLimitToggled() {
        return !(topLeftLim.get() && topRightLim.get()) && lastTopState;
    }

    /**
     * index balls in the hopper, but not the belts that feedd into the shooter
     */
    private void cycleIntake() {
        double currentBeltPosition = belt.getSensorCollection().getQuadraturePosition();
        double currentIndexerPosition = indexer.getSensorCollection().getQuadraturePosition();
        if (currentBeltPosition - initialBeltPosition < RobotMap.Hopper.ENC_COUNT_PER_CYCLE) {
            belt.set(RobotMap.Hopper.BELT_SPEED); // TODO: check motor speed with balls
        }
        else {        
            belt.set(0);
        }

        if (currentIndexerPosition - initialIndexerPosition < RobotMap.Hopper.ENC_COUNT_PER_CYCLE) {
            indexer.set(RobotMap.Hopper.INDEXER_SPEED); // TODO: check motor speed with balls
        }
        else {
            indexer.set(0);
        }
    }

    /**
     * called when the driver is ready to shoot (pushing the button on the controller)
     * sets the belt speed to the tested value necessary to feed 
     */
    public void shoot() {
        driverOverride = true;
        belt.set(RobotMap.Hopper.BELT_SPEED);
        indexer.set(RobotMap.Hopper.INDEXER_SPEED);
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
}
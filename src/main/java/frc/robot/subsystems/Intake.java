package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

/**
 * The Hopper subsystem consists of 3 motors to move the power cells in stages. Positions one and
 * two are on the bottom, position three is the transition to the upper level of the hopper, and
 * positions four and five are on the top level.
 *
 * TODO: This code is über messy and needs cleanup badly.
 * 
 * @author dri, JW, AF
 */

public class Intake {
    private static Intake instance;

    private DigitalInput botLeftLim;
    private DigitalInput botRightLim;

    // private DigitalInput midLeftLim;
    // private DigitalInput midRightLim;

    private DigitalInput topLeftLim;
    private DigitalInput topRightLim;

    private WPI_TalonSRX intake;
    private WPI_TalonSRX middle;
    private WPI_TalonSRX outtake;

    private final int MID_CYCLE_TIME = 100000; // in ms
    private final int CHAIN_CYCLE_TIME = 100000;

    private int count = 0;
    private int prev_count = 0;

    private Intake() {
        botLeftLim = new DigitalInput(RobotMap.Intake.Limit.BOTTOM_LEFT);
        botRightLim = new DigitalInput(RobotMap.Intake.Limit.BOTTOM_RIGHT);
        // midLeftLim = new DigitalInput(RobotMap.Intake.Limit.MIDDLE_LEFT);
        // midRightLim = new DigitalInput(RobotMap.Intake.Limit.MIDDLE_RIGHT);
        topLeftLim = new DigitalInput(RobotMap.Intake.Limit.TOP_LEFT);
        topRightLim = new DigitalInput(RobotMap.Intake.Limit.TOP_RIGHT);

        intake = new WPI_TalonSRX(RobotMap.Intake.Motor.INTAKE);
        middle = new WPI_TalonSRX(RobotMap.Intake.Motor.MIDDLE);
        outtake = new WPI_TalonSRX(RobotMap.Intake.Motor.OUTTAKE);
        intake.setInverted(RobotMap.Intake.Motor.INTAKE_IS_INVERTED);
        middle.setInverted(RobotMap.Intake.Motor.MIDDLE_IS_INVERTED);
        outtake.setInverted(RobotMap.Intake.Motor.OUTTAKE_IS_INVERTED);
    }

    /**
     * Returns a singular instance of the Intake subsystem.
     */
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    /**
     * Updates the current state of the turret. To be called in robotPeriodic().
     * TODO: "Add logic back in" (???)
     * TODO: Needs documentation, very obfuscated
     */
    public void update() {
        // adding a ball
        if (prev_count < count) {
            enableAndSleep();
            prev_count = count;
        } else if (prev_count > count) {  // want to remove a ball
            for (int i = 0; i<=5-count; i++) {
                enableAndSleep();
            }
            /* Ping that the shooter is ready, or set the shooter to delay:
             * CHAIN_CYCLE_TIME * i calculation (extra for loop)
             */
            count--;
            prev_count = count;
        }
    }

    /**
     * Enables the motors, sleeps, and then disables the motors.
     * TODO: is sleeping the thread necessary?
     */
    private boolean enableAndSleep() {
        intake.set(1);
        middle.set(1);
        outtake.set(1);
        Thread.sleep(MID_CYCLE_TIME/2);
        middle.set(0);
        Thread.sleep(CHAIN_CYCLE_TIME-MID_CYCLE_TIME/2);
        outtake.set(0);
        intake.set(0);
    }

    /**
     * Determine whether we are in a state where shooting is possible.
     */
    public boolean readyToShoot() {
        return top();
    }

    /**
     * Gets the current state of the bottom limit switches.
     * TODO: Should count be incremented here?
     */
    private boolean getBottomLimit() {
        count++;
        return botLeftLim.get() || botRightLim.get();
    }

    /**
     * Gets the current state of the top limit switches.
     */
    private boolean getTopLimit() {
        return topLeftLim.get() || topRightLim.get();
    }
}

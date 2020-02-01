package frc.robot.autonomous.paths;

/** 
 * Gets the robot to the baseline.
 * @author jk
 */

import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;

public class Baseline {
    private final double startPwr = 1; // TODO: untested
    private final double endPwr = 0; // TODO: untested

    /**
     * Drives straight until the given distance (in inches) specified in the constructor.
     */
    public void run() {
        while (Robot.drivetrain.getAveragePosition() < AutonMap.Baseline.DISTANCE) {
            Robot.drivetrain.driveStraight(AutonMap.Baseline.DISTANCE, startPwr, endPwr);
        }
    }

    /**
     * Stops the autonomous routine.
     */
    public void stop() {
        Robot.drivetrain.setSpeed(0, 0);
    }
 }
 
package frc.robot.autonomous.paths;

/** 
 * Gets the robot to the baseline.
 * @author jk
 */

import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;

public class Baseline {
    private double distance;

    /**
     * Constructs a new Baseline object.
     * @param distance the distance to travel in inches
     */
    private Baseline(double distance) {
        this.distance = distance;
    }

    private final double startPwr = 1; // TODO: untested
    private final double endPwr = 0; // TODO: untested

    /**
     * Drives straight until the given distance (in inches) specified in the constructor.
     */
    public void run() {
        while (Robot.drivetrain.getAveragePosition() < distance) {
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
 
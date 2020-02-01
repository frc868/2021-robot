package frc.robot.autonomous.paths;

/** 
 * Gets the robot to the baseline.
 * @author jk
 */

import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;

public class Baseline {
<<<<<<< HEAD
=======
    private double distance;

    /**
     * Constructs a new Baseline object.
     * @param distance the distance to travel in inches
     */
    private Baseline(double distance) {
        this.distance = distance;
    }

>>>>>>> c22ca42bc5cdc44d7841d7d2626a9c6265030e87
    private final double startPwr = 1; // TODO: untested
    private final double endPwr = 0; // TODO: untested

    /**
     * Drives straight until the given distance (in inches) specified in the constructor.
     */
    public void run() {
<<<<<<< HEAD
        while (Robot.drivetrain.getAveragePosition() < AutonMap.Baseline.DISTANCE) {
=======
        while (Robot.drivetrain.getAveragePosition() < distance) {
>>>>>>> c22ca42bc5cdc44d7841d7d2626a9c6265030e87
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
 
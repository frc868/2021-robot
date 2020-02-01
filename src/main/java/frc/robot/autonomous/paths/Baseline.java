package frc.robot.autonomous.paths;

import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;

/** 
 * Gets the robot to the baseline.
 * @author jk
 */
public class Baseline {
    /**
     * Runs the autonomous path.
     */
    public void run() {
        while (Robot.drivetrain.getAveragePosition() < AutonMap.Baseline.DISTANCE) {
            Robot.drivetrain.driveStraight(AutonMap.Baseline.DISTANCE, 
                                           AutonMap.Baseline.START_POWER,
                                           AutonMap.Baseline.END_POWER);
        }
    }

    /**
     * Stops the autonomous path.
     */
    public void stop() {
        Robot.drivetrain.setSpeed(0, 0);
    }
 }
 
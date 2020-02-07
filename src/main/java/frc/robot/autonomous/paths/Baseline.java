package frc.robot.autonomous.paths;

import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;

/** 
 * Gets the robot to the baseline.
 * @author jk, hrl
 */
public class Baseline {
    /**
     * Runs the autonomous path.
     */
    public void run() {
        Robot.drivetrain.driveStraight(AutonMap.Baseline.DISTANCE,
                                       AutonMap.Baseline.START_POWER,
                                       AutonMap.Baseline.END_POWER);
    }

    /**
     * Stops the autonomous path.
     */
    public void stop() {
        Robot.drivetrain.setSpeed(0, 0);
    }
 }

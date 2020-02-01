/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.paths;

import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;

/**
 * Backs up from the starting position and shoots three balls head-on.
 * @author hrl
 */
public class HeadOn {
    /**
     * Runs the autonomous path.
     */
    public void run() {
        // we're going backwards.
        while (Robot.drivetrain.getAveragePosition() > AutonMap.HeadOn.DISTANCE) {
            Robot.drivetrain.driveStraight(AutonMap.HeadOn.DISTANCE,
                                           AutonMap.HeadOn.START_POWER,
                                           AutonMap.HeadOn.END_POWER);
        }
        Robot.drivetrain.setSpeed(0, 0);

        // run the shooter
        while (Robot.hopper.getBallCount() > 0) {
            Robot.hopper.shoot();
            // TODO: this should be PID-controlled, waiting on shooter group...
            Robot.shooter.setSpeed(AutonMap.HeadOn.SHOOTER_POWER);
        }
        Robot.hopper.stop();
    }

    /**
     * Stops the autonomous path.
     */
    public void stop() {
        Robot.hopper.stop();
        Robot.shooter.stop();
        Robot.drivetrain.setSpeed(0, 0);
    }
}

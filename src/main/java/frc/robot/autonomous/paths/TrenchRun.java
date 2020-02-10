/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.paths;

import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;
import frc.robot.autonomous.TurnToAngleGyro;

/**
 * Goes off the starting line, picks up two balls, turns, goes towards the goal,
 * and shoots all five (hopefully into the inner).
 * @author hrl
 */
public class TrenchRun {
    /**
     * Runs the autonomous path.
     */
    public void run() {
        // drive to the balls
        Robot.drivetrain.driveStraight(AutonMap.TrenchRun.DISTANCE_TO_BALLS,
                                       AutonMap.TrenchRun.START_POWER,
                                       AutonMap.TrenchRun.END_POWER);

        // grab the balls
        Robot.intake.intakeUntilFull(AutonMap.TrenchRun.INTAKE_DELAY,
                                     AutonMap.TrenchRun.INTAKE_POWER);

        // turn
        new TurnToAngleGyro(AutonMap.TrenchRun.TURN_ANGLE).run();

        // drive to the target
        Robot.drivetrain.driveStraight(AutonMap.TrenchRun.DISTANCE_TO_TARGET,
                                       AutonMap.TrenchRun.START_POWER,
                                       AutonMap.TrenchRun.END_POWER);

        /* TODO: check if turret is really on target. should be running constantly in
           autonPeriodic() unless we're doing baseline auto. */

        // shoot 'em up
        Robot.shooter.shootUntilClear(AutonMap.TrenchRun.SHOOTER_POWER);
    }

    /**
     * Stops the autonomous path.
     */
    public void stop() {
        Robot.drivetrain.setSpeed(0, 0);
        Robot.intake.setSpeed(0);
        Robot.hopper.stop();
        Robot.shooter.stop();
    }
}

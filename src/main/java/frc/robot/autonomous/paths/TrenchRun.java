/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.paths;

/**
 * Goes off the starting line, picks up two balls, turns, goes towards the goal,
 * and shoots all five (hopefully into the inner).
 * @author hrl
 */
import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;
import frc.robot.autonomous.TurnToAngleGyro;

public class TrenchRun {
    /**
     * Runs the autonomous path.
     */
    public void run() {
        // drive to the balls
        while (Robot.drivetrain.getAveragePosition() < AutonMap.TrenchRun.DISTANCE_TO_BALLS) {
            Robot.drivetrain.driveStraight(AutonMap.TrenchRun.DISTANCE_TO_BALLS, 
                                           AutonMap.TrenchRun.START_POWER,
                                           AutonMap.TrenchRun.END_POWER);
        }
        Robot.drivetrain.setSpeed(0, 0);

        // grab the balls
        while (Robot.hopper.getBallCount() < 5) {
            Robot.intake.setSpeed(AutonMap.TrenchRun.INTAKE_POWER);
        }
        Robot.intake.setSpeed(0);

        // turn
        new TurnToAngleGyro(AutonMap.TrenchRun.TURN_ANGLE).run();

        // drive to the target
        while (Robot.drivetrain.getAveragePosition() < AutonMap.TrenchRun.DISTANCE_TO_TARGET) {
            Robot.drivetrain.driveStraight(AutonMap.TrenchRun.DISTANCE_TO_TARGET,
                                           AutonMap.TrenchRun.START_POWER,
                                           AutonMap.TrenchRun.END_POWER);
        }
        Robot.drivetrain.setSpeed(0, 0);

        /* TODO: check if turret is really on target. should be running constantly in
           autonPeriodic() unless we're doing baseline auto. */

        // shoot 'em up
        while (Robot.hopper.getBallCount() > 0) {
            Robot.hopper.shoot();
            // TODO: this should be PID-controlled, waiting on shooter group...
            Robot.shooter.setSpeed(AutonMap.TrenchRun.SHOOTER_POWER);
        }
        Robot.hopper.stop();
        Robot.shooter.setSpeed(0);
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
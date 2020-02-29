/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.paths;

import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;
import frc.robot.autonomous.AutonPath;
import frc.robot.autonomous.TurnToAngleGyro;

/**
 * Goes off the starting line, picks up two balls, turns, goes towards the goal,
 * and shoots all five (hopefully into the inner).
 * @author hrl
 */
public class TrenchRun extends AutonPath {
    private static double currentDistance = 0;
    private static int currentBallCount = 0;
    private TrenchRunState currentState = TrenchRunState.ToBallIntake;

    private static TurnToAngleGyro ttag = new TurnToAngleGyro();

    private enum TrenchRunState {
        ToBallIntake {
            @Override
            public TrenchRunState nextState() {
                if (currentDistance < AutonMap.TrenchRun.DISTANCE_TO_BALLS) {
                    return this;
                }
                return Intaking;
            }

            @Override
            public void run() {
                Robot.drivetrain.driveStraight(AutonMap.TrenchRun.DISTANCE_TO_BALLS,
                                               AutonMap.TrenchRun.START_POWER,
                                               AutonMap.TrenchRun.END_POWER);
            }
        },
        Intaking {
            @Override
            public TrenchRunState nextState() {
                // intakeUntilFull() returns true if the hopper is full or the time delay has
                // been crossed.
                if (Robot.intake.intakeUntilFull(AutonMap.TrenchRun.INTAKE_DELAY,
                                                 AutonMap.TrenchRun.INTAKE_POWER)) {
                    return Turning;
                }
                return this;
            }

            @Override
            public void run() {
                // NOTE: purposefully a stub. nextState actually runs the intake code.
                ttag.reset(); // NOTE: needed for TTAG to work without a ReadyToTurn state.
                Robot.drivetrain.setSpeed(0, 0);
            }
        },
        Turning {
            @Override
            public TrenchRunState nextState() {
                if (Robot.gyro.getAngle() < AutonMap.TrenchRun.TURN_ANGLE) {
                    return this;
                }
                return ReadyToShootPosition;
            }

            @Override
            public void run() {
                Robot.intake.setSpeed(0); // TODO: could be part of an intake transitional stage?
                ttag.run(AutonMap.TrenchRun.TURN_ANGLE);
            }
        },
        ReadyToShootPosition {
            @Override
            public TrenchRunState nextState() {
                // NOTE: this only needs to run once
                return ToShootPosition;
            }

            @Override
            public void run() {
                Robot.drivetrain.resetInitialDistance();
            }

        },
        ToShootPosition {
            @Override
            public TrenchRunState nextState() {
                if (currentDistance < AutonMap.TrenchRun.DISTANCE_TO_TARGET) {
                    return this;
                }
                return Shooting;
            }

            @Override
            public void run() {
                Robot.drivetrain.driveStraight(AutonMap.TrenchRun.DISTANCE_TO_TARGET,
                                               AutonMap.TrenchRun.START_POWER,
                                               AutonMap.TrenchRun.END_POWER);
            }
        },
        Shooting {
            @Override
            public TrenchRunState nextState() {
                if (currentBallCount > 0) {
                    return this;
                }
                return Done;
            }

            @Override
            public void run() {
                Robot.drivetrain.setSpeed(0, 0);
                Robot.shooter.shootUntilClear(AutonMap.TrenchRun.SHOOTER_POWER);
            }
        },
        Done {
            @Override
            public TrenchRunState nextState() {
                return this;
            }

            @Override
            public void run() {
                Robot.drivetrain.setSpeed(0, 0);
                Robot.intake.setSpeed(0);
                Robot.hopper.stop();
                Robot.shooter.stop();
            }
        };

        public abstract TrenchRunState nextState();
        public abstract void run();
    }

    /**
     * Runs the autonomous path.
     */
    @Override
    public void run() {
        // update state variables
        currentDistance = Robot.drivetrain.getCurrentDistance();
        currentBallCount = Robot.hopper.getBallCount();

        this.currentState.run();
        this.currentState = this.currentState.nextState();
    }

    @Override
    public String toString() {
        return "Trench Run";
    }
}

package frc.robot.autonomous.paths;

import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;
import frc.robot.autonomous.AutonPath;

/**
 * God help us all.
 * @author hrl
 */
public class SixBall extends AutonPath {
    private static double currentDistance = 0;
    private static int currentBallCount = 0;
    private SixBallState currentState = SixBallState.Shooting;

    private enum SixBallState {
        Shooting {
            @Override
            public SixBallState nextState() {
                if (currentBallCount > 0) {
                    return this;
                }
                return Done;
            }

            @Override
            public void run() {
                Robot.shooter.shootUntilClear(AutonMap.SixBall.SHOOTER_POWER);
            }
        },
        DriveToBalls {
            @Override
            public SixBallState nextState() {
                if (currentDistance < AutonMap.SixBall.DISTANCE_TO_BALLS) {
                    return this;
                }
                return ReadyToIntake;
            }

            @Override
            public void run() {
                Robot.drivetrain.driveStraight(AutonMap.SixBall.DISTANCE_TO_BALLS,
                                               AutonMap.SixBall.START_POWER,
                                               AutonMap.SixBall.END_POWER);
                Robot.hopper.stop();
                Robot.shooter.stop();
            } 
        },
        ReadyToIntake {
            @Override
            public SixBallState nextState() {
                return Intaking;
            }

            @Override
            public void run() {
                Robot.drivetrain.resetInitialDistance();
            }
        },
        Intaking {
            @Override
            public SixBallState nextState() {
                if (Robot.intake.intakeUntilFull(AutonMap.SixBall.INTAKE_DELAY,
                                                 AutonMap.SixBall.INTAKE_POWER)) {
                    return ReadyToDrive;
                }
                return this;
            }

            @Override
            public void run() {
                Robot.drivetrain.driveStraight(AutonMap.SixBall.DISTANCE_WHILE_BALLS,
                                               AutonMap.SixBall.START_POWER,
                                               AutonMap.SixBall.END_POWER);
            }
        },
        ReadyToDrive {
            @Override
            public SixBallState nextState() {
                return DriveToTarget;
            }

            @Override
            public void run() {
                Robot.drivetrain.resetInitialDistance();
                Robot.intake.setSpeed(0);
            }
        },
        DriveToTarget {
            @Override
            public SixBallState nextState() {
                if (currentDistance < AutonMap.TrenchRun.DISTANCE_TO_BALLS) {
                    return this;
                }
                return ShootingAgain;
            }

            @Override
            public void run() {
                // backwards...
                Robot.drivetrain.driveStraight(AutonMap.SixBall.DISTANCE_TO_BALLS +
                                               AutonMap.SixBall.DISTANCE_WHILE_BALLS,
                                               -AutonMap.SixBall.START_POWER,
                                               AutonMap.SixBall.END_POWER);
            }
        },
        ShootingAgain {
            @Override
            public SixBallState nextState() {
                if (currentBallCount > 0) {
                    return this;
                }
                return Done;
            }

            @Override
            public void run() {
                Robot.drivetrain.setSpeed(0, 0);
                Robot.shooter.shootUntilClear(AutonMap.SixBall.SHOOTER_POWER);
            }
        },
        Done {
            @Override
            public SixBallState nextState() {
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

        public abstract SixBallState nextState();
        public abstract void run();
    }

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
        return "Six Ball";
    }
}

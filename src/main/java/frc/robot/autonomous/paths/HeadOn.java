/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;
import frc.robot.autonomous.AutonPath;

/**
 * Backs up from the starting position and shoots three balls head-on.
 * @author hrl
 */
public class HeadOn extends AutonPath {
    private static double currentDistance = 0;
    private static double currentVelocity = 0;
    private static Timer shootDelay = new Timer();
    private static double setpoint = AutonMap.HeadOn.SHOOTER_RPM;
    private static double startPower = AutonMap.HeadOn.START_POWER;
    private HeadOnState currentState = HeadOnState.ToShootPosition;

    // TODO: refactor this to not do dead reckoning at bloomington
    private enum HeadOnState {
        ToShootPosition {
            @Override
            public HeadOnState nextState() {
                // tolerance
                if (currentDistance < AutonMap.HeadOn.DISTANCE - 1) {
                    return this;
                }
                return ReadyToReady;
            }

            @Override
            public void run() {
                Robot.drivetrain.driveStraight(AutonMap.HeadOn.DISTANCE, AutonMap.HeadOn.START_POWER,
                        AutonMap.HeadOn.END_POWER);
                shootDelay.start();
            }

            @Override
            public String toString() {
                return "Moving";
            }
        },
        ReadyToReady {
            @Override
            public HeadOnState nextState() {
                return Readying;
            }

            @Override
            public void run() {
                shootDelay.start();
            }

            @Override
            public String toString() {
                return "Setting timer";
            }
        },
        Readying {
            @Override
            public HeadOnState nextState() {
                // TODO: this should check velocity of shooter
                if (shootDelay.get() < 3) {
                    return this;
                }
                return ReadyToShoot;
            }

            @Override
            public void run() {
                Robot.drivetrain.setSpeed(0, 0);
                Robot.shooter.setSpeed(AutonMap.HeadOn.SHOOTER_POWER);
                // Robot.shooter.update(setpoint);
            }

            @Override
            public String toString() {
                return "Readying";
            }
        },
        ReadyToShoot {
            @Override
            public HeadOnState nextState() {
                return Shooting;
            }

            @Override
            public void run() {
                shootDelay.reset();
            }

            @Override
            public String toString() {
                return "Setting timer";
            }
        },
        Shooting {
            @Override
            public HeadOnState nextState() {
                if (shootDelay.get() > 5) {
                    return Done;
                }
                return this;
            }

            @Override
            public void run() {
                Robot.drivetrain.setSpeed(0, 0);
                Robot.shooter.setSpeed(AutonMap.HeadOn.SHOOTER_POWER);
                Robot.hopper.forward();
                // Robot.shooter.shootUntilClear(AutonMap.HeadOn.SHOOTER_RPM);
            }

            @Override
            public String toString() {
                return "Shooting";
            }
        },
        Done {
            @Override
            public HeadOnState nextState() {
                return this;
            }

            @Override
            public void run() {
                Robot.hopper.stop();
                Robot.shooter.stop();
                Robot.drivetrain.setSpeed(0, 0);
            }

            @Override
            public String toString() {
                return "Done";
            }
        };

        public abstract HeadOnState nextState();
        public abstract void run();
        public abstract String toString();
    }

    public HeadOn(double rpm, boolean isInverted) {
        setpoint = rpm;
        if (isInverted) {
            startPower = -AutonMap.HeadOn.START_POWER;
        }
    }

    public HeadOn(boolean isInverted) {
        if (isInverted) {
            startPower = -AutonMap.HeadOn.START_POWER;
        }
    }

    public HeadOn() {
        // NOTE: intentionally a stub
    }

    /**
     * Runs the autonomous path.
     */
    @Override
    public void run() {
        // update state variables
        currentDistance = Robot.drivetrain.getCurrentDistance();
        currentVelocity = Robot.shooter.getRPM();

        SmartDashboard.putString("Auton state", this.currentState.toString());
        this.currentState.run();
        this.currentState = this.currentState.nextState();
    }

    @Override
    public void reset() {
        this.currentState = HeadOnState.ToShootPosition;
    }

    @Override
    public String toString() {
        return "Head-on";
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.paths;

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
    private static int currentBallCount = 0;
    private HeadOnState currentState = HeadOnState.ToShootPosition;

    private enum HeadOnState {
        ToShootPosition {
            @Override
            public HeadOnState nextState() {
                if (currentDistance < AutonMap.HeadOn.DISTANCE-1) {
                    return this;
                }
                return ReadyToShoot;
            }

            @Override
            public void run() {
                Robot.drivetrain.driveStraight(AutonMap.HeadOn.DISTANCE, AutonMap.HeadOn.START_POWER,
                        AutonMap.HeadOn.END_POWER);
            }
        },
        ReadyToShoot {
            @Override
            public HeadOnState nextState() {
                if (currentVelocity < AutonMap.HeadOn.SHOOTER_RPM) {
                    return this;
                }
                return Shooting;
            }

            @Override
            public void run() {
                Robot.drivetrain.setSpeed(0, 0);
                Robot.shooter.update();
            }
        },
        Shooting {
            @Override
            public HeadOnState nextState() {
                return this;
            }

            @Override
            public void run() {
                Robot.drivetrain.setSpeed(0, 0);
                Robot.shooter.shootUntilClear(AutonMap.HeadOn.SHOOTER_RPM);
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
        };

        public abstract HeadOnState nextState();
        public abstract void run();
    }

    /**
     * Runs the autonomous path.
     */
    @Override
    public void run() {
        // update state variables
        currentDistance = Robot.drivetrain.getCurrentDistance();
        //currentBallCount = Robot.hopper.getBallCount();
        currentVelocity = Robot.shooter.getRPM();

        switch(this.currentState) {
        case ToShootPosition:
            SmartDashboard.putString("Auton state", "To shoot pos");
            break;
        case ReadyToShoot:
            SmartDashboard.putString("Auton state", "Readying");
            break;
        case Shooting:
            SmartDashboard.putString("Auton state", "Shooting");
            break;
        default:
            SmartDashboard.putString("Auton state", "What");
            break;
        }

        this.currentState.run();
        this.currentState = this.currentState.nextState();
    }

    @Override
    public String toString() {
        return "Head-on";
    }
}

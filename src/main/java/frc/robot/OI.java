package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helpers.ControllerWrapper;

/**
 * The class in which we map our driver/operator input to specific tasks on the
 * robot Init should be called once in the robotInit() method in the Robot class
 * Update should be called either in robotPeriodic() or teleopPeriodic()
 * @author hrl
 */
public class OI {
    public static ControllerWrapper driver = new ControllerWrapper(RobotMap.Controllers.DRIVER_PORT, true);
    public static ControllerWrapper operator = new ControllerWrapper(RobotMap.Controllers.OPERATOR_PORT, true);

    public static void init() {
        initSD();
        Robot.shooter.init();
    }

    public static void update() {
        // GENERAL CONTROLS/CONTROL METHODS
        Robot.turret.manualTurret();

        // DRIVER CONTROLS
        driver.bMENU.whenPressed(() -> {
            driver.toggleAltMode();
            operator.toggleAltMode();
        });
        driver.bMENU.whenReleased(() -> {
            Robot.climber.stopWinch();
            Robot.climber.engageBrake();
        });

        if (driver.isAltMode()) {
            Robot.drivetrain.arcadeDrive(0.4);
        } else {
            Robot.drivetrain.arcadeDrive(1);
        }

        // turret
        driver.bRB.whileHeld(Robot.turret::trackVision);
        driver.bRB.whenReleased(Robot.turret::stop);

        operator.bY.whileHeld(() -> Robot.climber.manualArm(0.1));
        operator.bY.whenReleased(Robot.climber::stopArm);
        driver.bA.whileHeld(() -> Robot.climber.manualArm(-0.1));
        driver.bA.whenReleased(Robot.climber::stopArm);

        // pt 3 testing
        driver.bB.whenPressed(() -> {
            Robot.climber.disengageBrake();
            Robot.climber.moveArmUp(0.1);
            Robot.climber.engageBrake();
        });

        // OPERATOR CONTROLS
        // set the operator mode state
        operator.bMENU.whenPressed(() -> {
            driver.toggleAltMode();
            operator.toggleAltMode();
        });
        operator.bMENU.whenReleased(() -> {
            Robot.climber.stopWinch();
            Robot.climber.engageBrake();
        });

        if (operator.isAltMode()) {
            System.out.println("=== ALT MODE ENABLED ===");
            operator.trigLSTK.whileHeld(() -> {
                Robot.climber.manualClimb(RobotMap.Climber.HOLD_POWER);
            });
            operator.trigRSTK.whileHeld(() -> {
                Robot.climber.setSpeedArm(RobotMap.Climber.ARM_POWER);
            });

            // operator.trigLSTK.whenPressed(() -> {
            //     if (operator.getLY() > 0) {
            //         Robot.climber.disengageBrake();
            //         Robot.climber.setSpeedWinch(0.3); // TODO Test value
            //     } else if (operator.getLY() < 0) {
            //         Robot.climber.disengageBrake();
            //         Robot.climber.setSpeedWinch(-0.3);
            //     } else {
            //         Robot.climber.stopWinch();
            //         Robot.climber.engageBrake();
            //     }
            // });
            // operator.trigRSTK.whenPressed(() -> {
            //     if (operator.getRY() > 0) {
            //         Robot.climber.setSpeedArm(0.1);
            //     } else if (operator.getRY() < 0) {
            //         Robot.climber.setSpeedArm(-0.1);
            //     } else {
            //         Robot.climber.setSpeedArm(0);
            //     }
            // });
            // TODO: check these 2020-03-07
            operator.dN.whenPressed(() -> {
                Robot.climber.moveArmUp(0.1);
            });
            operator.dS.whenReleased(() -> {
                Robot.climber.moveArmDown(-0.1);
                Robot.climber.activateWinch();
            });
            // operator.dN.whenPressed(() -> {
            //     Robot.climber.disengageBrake();
            //     Robot.climber.setSpeedWinch(0.3);
            // });
            // operator.dN.whenReleased(() -> {
            //     Robot.climber.stopWinch();
            //     Robot.climber.engageBrake();
            // });
            // operator.dS.whenPressed(() -> {
            //     Robot.climber.disengageBrake();
            //     Robot.climber.setSpeedWinch(-0.3);
            // });
            // operator.dS.whenReleased(() -> {
            //     Robot.climber.stopWinch();
            //     Robot.climber.engageBrake();
            // });
        } else {
            // turret
            operator.bRB.whileHeld(Robot.turret::trackVision);
            operator.bRB.whenReleased(Robot.turret::stop);

            // shoot
            operator.bA.whileHeld(() -> Robot.shooter.setSpeed(0.6));
            operator.bA.whenReleased(Robot.shooter::stop);
            operator.bSTART.whileHeld(() -> Robot.hopper.forward(Robot.shooter.atTarget()));
            operator.bSTART.whenReleased(Robot.hopper::stop);

            // intake
            operator.bY.whenPressed(Robot.intake::toggle);

            operator.bRT.whileHeld(() -> {
                Robot.hopper.update();
                Robot.intake.setSpeed(1);
            });
            operator.bRT.whenReleased(() -> {
                Robot.hopper.stop();
                Robot.intake.setSpeed(0);
            });
            operator.bLT.whileHeld(() -> {
                Robot.hopper.reverse(.6);
                Robot.intake.setSpeed(-1);
            });
            operator.bLT.whenReleased(() -> {
                Robot.hopper.stop();
                Robot.intake.setSpeed(0);
            });

            // hopper
            operator.bB.whileHeld(() -> Robot.hopper.reverse(.6));
            operator.bB.whenReleased(Robot.hopper::stop);

            // WOF
            operator.dN.whenPressed(Robot.wheel::actuatorUp);
            operator.dS.whenPressed(Robot.wheel::actuatorDown);
        }

        // if it hasn't already been handled...
        driver.updateStates();
        operator.updateStates();

        // climber
        // operator.bX.whileHeld(() -> Robot.climber.testWinch());
        // operator.bX.whenReleased(() -> Robot.climber.testWinch());

        updateSD();
    }

    public static void initSD() {
    }

    public static void updateSD() {
        // SmartDashboard.putString("WoF Color", Robot.wheel.toString());
        // SmartDashboard.putBoolean("Bot Sensor", Robot.hopper.getBotSensor());
        // SmartDashboard.putBoolean("Mid Sensor", Robot.hopper.getMidLimit());
        // SmartDashboard.putBoolean("Top Sensor", Robot.hopper.getTopLimit());
        SmartDashboard.putNumber("CL_Winch", Robot.climber.getWinchPosition());
        SmartDashboard.putNumber("CL_Arm", Robot.climber.getArmPosition());
        SmartDashboard.putBoolean("CL_Sensor", Robot.climber.getArmDeploy());
        // SmartDashboard.putNumber("Hopper count", Robot.hopper.getBallCount());

        // SmartDashboard.putBoolean("Turret left", Robot.turret.getLeftLimit());
        // SmartDashboard.putBoolean("Turret right", Robot.turret.getRightLimit());

        // SmartDashboard.putNumber("Left trigger", operator.getLT());
        // SmartDashboard.putNumber("Right trigger", operator.getRT());
    }
}

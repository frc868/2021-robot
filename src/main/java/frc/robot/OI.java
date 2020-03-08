package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helpers.ControllerWrapper;

/**
 * The class in which we map our driver/operator input to specific tasks on the
 * robot Init should be called once in the robotInit() method in the Robot class
 * Update should be called either in robotPeriodic() or teleopPeriodic()
 * 
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
        Robot.drivetrain.arcadeDrive(1);
        // TODO: change manual turret to joystick
        // TESTING TODO: delete these after they're tested
        // operator.bLB.whenPressed(() -> Robot.turret.safeZone());
        // -----------------------------------------------
        Robot.turret.manualTurret();
        // Robot.climber.manualArm();

        // DRIVER CONTROLS
        driver.bMENU.whenPressed(() -> {
            driver.toggleAltMode();
            operator.toggleAltMode();
        });
        driver.bMENU.whenReleased(() -> {
            Robot.climber.stopWinch();
            Robot.climber.engageBrake();
        });

        // shoot
        driver.bA.whileHeld(() -> Robot.shooter.setSpeed(0.6));
        driver.bA.whenReleased(Robot.shooter::stop);
        driver.bSTART.whileHeld(() -> Robot.hopper.forward(Robot.shooter.atTarget()));
        driver.bSTART.whenReleased(Robot.hopper::stop);

        // hopper
        driver.bB.whileHeld(() -> Robot.hopper.reverse(.6));
        driver.bB.whenReleased(Robot.hopper::stop);

        if (driver.isAltMode()) {
            Robot.drivetrain.arcadeDrive(0.4);
        } else {
            Robot.drivetrain.arcadeDrive(1);
        }

        // turret
        /* operator.bLB.whileHeld(() -> */ Robot.turret.manualTurret();/* ); */
        // operator.bLB.whenReleased(() -> {});
        operator.bRB.whileHeld(() -> Robot.turret.trackVision());
        operator.bRB.whenReleased(() -> Robot.turret.stop());

        // shoot
        operator.bSTART.whileHeld(() -> Robot.shooter.setSpeed(0.6));
        operator.bSTART.whenReleased(() -> Robot.shooter.stop());
        
        driver.bRB.whileHeld(Robot.turret::trackVision);
        driver.bRB.whenReleased(Robot.turret::stop);

        // working manual arm

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
        operator.bY.whenPressed(() -> Robot.climber.deployHook());
        operator.bY.whenReleased(() -> Robot.climber.stopArm());


        if (operator.isAltMode()) {
            System.out.println("=== ALT MODE ENABLED ===");
            operator.bA.whenPressed(() -> {
                // Robot.climber.disengageBrake();
                Robot.climber.setSpeedArm(-0.3);
            });
            operator.bA.whenReleased(() -> Robot.climber.stopArm());
            // operator.bA.whileHeld(() -> Robot.climber.manualArm(-0.3));
            // operator.bA.whenReleased(Robot.climber::stopArm);
            // ===========================================================
            // working manual arm
            // operator.bY.whileHeld(() -> Robot.climber.manualArm(0.3));
            // operator.bY.whenReleased(Robot.climber::stopArm);
            //
            // operator.trigLSTK.whileHeld(() -> {
            // Robot.climber.manualClimb(RobotMap.Climber.HOLD_POWER);
            // });
            // operator.trigRSTK.whileHeld(() -> {
            // Robot.climber.setSpeedArm(RobotMap.Climber.ARM_POWER);
            // });

            // operator.trigRSTK.whenReleased(() -> {
            // Robot.climber.stopArm();
            // });
            // operator.trigLSTK.whenReleased(() -> {
            // Robot.climber.stopWinch();
            // });

            // operator.trigLSTK.whenPressed(() -> {
            // if (operator.getLY() > 0) {
            // Robot.climber.disengageBrake();
            // Robot.climber.setSpeedWinch(0.3); // TODO Test value
            // } else if (operator.getLY() < 0) {
            // Robot.climber.disengageBrake();
            // Robot.climber.setSpeedWinch(-0.3);
            // } else {
            // Robot.climber.stopWinch();
            // Robot.climber.engageBrake();
            // }
            // });
            // operator.trigRSTK.whenPressed(() -> {
            // if (operator.getRY() > 0) {
            // Robot.climber.setSpeedArm(0.1);
            // } else if (operator.getRY() < 0) {
            // Robot.climber.setSpeedArm(-0.1);
            // } else {
            // Robot.climber.setSpeedArm(0);
            // }
            // });
            // TODO: check these 2020-03-07

            // auto mode
            // operator.dN.whenPressed(() -> {
            // Robot.climber.moveArmUp(0.1);
            // });
            // operator.dS.whenReleased(() -> {
            // Robot.climber.moveArmDown(-0.1);
            // Robot.climber.activateWinch();
            // });

            // working winch
            operator.dN.whenPressed(() -> {
                Robot.climber.disengageBrake();
                Robot.climber.setSpeedWinch(0.5);
            });
            operator.dN.whenReleased(() -> {
                Robot.climber.stopWinch();
                Robot.climber.engageBrake();
            });
            operator.dS.whenPressed(() -> {
                Robot.climber.disengageBrake();
                Robot.climber.setSpeedWinch(-0.5);
            });
            operator.dS.whenReleased(() -> {
                Robot.climber.stopWinch();
                Robot.climber.engageBrake();
            });

            Robot.turret.stop();
        } else {
            // turret
            operator.bRB.whileHeld(Robot.turret::trackVision);
            operator.bRB.whenReleased(Robot.turret::stop);

            // // shoot
            // operator.bA.whileHeld(() -> Robot.shooter.setSpeed(0.6));
            // operator.bA.whenReleased(Robot.shooter::stop);
            // operator.bSTART.whileHeld(() ->
            // Robot.hopper.forward(Robot.shooter.atTarget()));
            // operator.bSTART.whenReleased(Robot.hopper::stop);

            // intake
            // operator.bY.whenPressed(Robot.intake::toggle);
            operator.bA.whileHeld(() -> {
                // Robot.shooter.update(4300);
                Robot.shooter.update(Robot.camera.getCalculatedRPM()); // TODO: needs testing
                Robot.hopper.forward(Robot.shooter.atTarget());
            });
            operator.bA.whenReleased(() -> {
                Robot.shooter.stop();
                Robot.hopper.stop();
            });

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
        SmartDashboard.putBoolean("Bot Left Sensor", Robot.hopper.getBotLeftSensor());
        SmartDashboard.putBoolean("Bot Right Sensor", Robot.hopper.getBotRightSensor());

        SmartDashboard.putBoolean("Mid Sensor", Robot.hopper.getMidLimit());
        SmartDashboard.putBoolean("Top Sensor", Robot.hopper.getTopLimit());
        SmartDashboard.putNumber("Hopper count", Robot.hopper.getBallCount());

        SmartDashboard.putBoolean("Turret left", Robot.turret.getLeftLimit());
        SmartDashboard.putBoolean("Turret right", Robot.turret.getRightLimit());

        SmartDashboard.putNumber("Turret Speed", Robot.turret.getSpeed());
        SmartDashboard.putNumber("Limelight Stuff ????", Robot.camera.getPosition());

        SmartDashboard.putNumber("Left trigger", operator.getLT());
        SmartDashboard.putNumber("Right trigger", operator.getRT());

        SmartDashboard.putBoolean("Shooter running?", Robot.shooter.getRPM() > 0);
        SmartDashboard.putBoolean("At target", Robot.shooter.atTarget());

        SmartDashboard.putNumber("CL_Winch", Robot.climber.getWinchPosition());
        SmartDashboard.putNumber("CL_Arm", Robot.climber.getArmPosition());
        SmartDashboard.putBoolean("CL_Sensor", Robot.climber.getArmDeploy());

        SmartDashboard.putNumber("DT Left Enc", Robot.drivetrain.getLeftPosition());
    }
}

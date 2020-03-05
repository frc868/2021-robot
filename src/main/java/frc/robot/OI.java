package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helpers.ControllerWrapper;
import frc.robot.helpers.Helper;

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
    public static boolean armReset = false;
    public static boolean engaged = false;

    public static void init() {
        Robot.shooter.init();
    }

    public static void update() {
        // HUGE MEGA TODO: figure out controls with driver and operator
        // GENERAL CONTROLS/CONTROL METHODS
        Robot.drivetrain.arcadeDrive(1);
        Robot.turret.manualTurret();
        
        //TODO: change manual turret to joystick

        // DRIVER CONTROLS
        // turret
        driver.bRB.whileHeld(Robot.turret::trackVision);
        driver.bRB.whenReleased(Robot.turret::stop);

        driver.bMENU.whenPressed(operator::toggleAltMode);

        // OPERATOR CONTROLS
        // set the operator mode state
        operator.bMENU.whenPressed(operator::toggleAltMode);

        if (operator.isAltMode()) {
        } else {
            // turret
            operator.bRB.whileHeld(Robot.turret::trackVision);
            operator.bRB.whenReleased(Robot.turret::stop);

            // shoot
            operator.bA.whileHeld(() -> Robot.shooter.setSpeed(0.6));
            operator.bA.whenReleased(Robot.shooter::stop);
            operator.bSTART.whileHeld(Robot.hopper::forward);
            operator.bSTART.whenReleased(() -> {
                Robot.hopper.stop();
                Robot.hopper.resetOverride();
            });

            // intake
            operator.bY.whenPressed(Robot.intake::toggle);

            Robot.intake.setSpeed(
                    Helper.analogToDigital(operator.getRT(), .1, 1) - Helper.analogToDigital(operator.getLT(), .1, 1));
            operator.bRT.whileHeld(() -> {
                Robot.hopper.update();
                Robot.intake.setSpeed(1);
            });
            operator.bLT.whileHeld(() -> {
                Robot.hopper.reverse(.6);
                Robot.intake.setSpeed(-1);
            });
            operator.bRT.whenReleased(() -> {
                Robot.hopper.stop();
                Robot.intake.setSpeed(0);
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

        //only for testing - pt 1
        driver.bX.whenPressed(() -> Robot.climber.engageBrake());
        driver.bY.whenPressed(() -> Robot.climber.disengageBrake());
        driver.bA.whileHeld(() -> Robot.climber.testWinch());

        // pt 2 testing
        driver.bA.whileHeld(() -> {
            Robot.climber.setEngaged(false);
            Robot.climber.testWinch();
        });
        driver.bA.whenReleased(() -> {
            Robot.climber.setEngaged(true);
            Robot.climber.disengageBrake();
        });

        //pt 3 testing
        driver.bA.whenPressed(() -> {
            Robot.climber.disengageBrake();
            Robot.climber.moveArmUp(0, 0); //TODO: set parameters
            Robot.climber.engageBrake();
        });




        updateSD();

        if (armReset == false) {
            Robot.climber.resetArmPosition();;
            armReset = true;
        }

        
    }

    public static void updateSD() {
        SmartDashboard.putString("WoF Color", Robot.wheel.toString());
        SmartDashboard.putBoolean("Bot Sensor", Robot.hopper.getBotSensor());
        SmartDashboard.putBoolean("Mid Sensor", Robot.hopper.getMidLimit());
        SmartDashboard.putBoolean("Top Sensor", Robot.hopper.getTopLimit());
        SmartDashboard.putNumber("CL_deploy", Robot.climber.getArmPosition());

        SmartDashboard.putBoolean("Turret left", Robot.turret.getLeftLimit());
        SmartDashboard.putBoolean("Turret right", Robot.turret.getRightLimit());

        SmartDashboard.putNumber("Left trigger", operator.getLT());
        SmartDashboard.putNumber("Right trigger", operator.getRT());
    }
}

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helpers.ControllerWrapper;
import frc.robot.helpers.Helper;

/**
 * The class in which we map our driver/operator input to specific tasks on the robot
 * Init should be called once in the robotInit() method in the Robot class
 * Update should be called either in robotPeriodic() or teleopPeriodic()
 * @author hrl
 */

public class OI {
    public static ControllerWrapper driver = new ControllerWrapper(RobotMap.Controllers.DRIVER_PORT, true);
    public static ControllerWrapper operator = new ControllerWrapper(RobotMap.Controllers.OPERATOR_PORT, true);

    public static void init() {

    }

    public static void update() {
        // HUGE MEGA TODO: figure out controls with driver and operator
        // GENERAL CONTROLS/CONTROL METHODS
        Robot.drivetrain.arcadeDrive(1);
        Robot.turret.manualTurret();
        //TODO: change manual turret to joystick

        // DRIVER CONTROLS

        // OPERATOR CONTROLS
        // set the operator mode state
        operator.bMENU.whenPressed(operator::toggleAltMode);

        if (operator.isAltMode()) {
            // climber controls
            operator.dN.whenPressed(Robot.climber::moveArmUp);
            operator.dS.whenPressed(() -> {
                Robot.climber.moveArmDown();
                Robot.climber.activateWinch();
            });
        } else {
            // shoot
            operator.bA.whenPressed(() -> Robot.shooter.setSpeed(-0.6));
            operator.bX.whenPressed(Robot.shooter::stop);
            operator.bSTART.whileHeld(() -> Robot.hopper.forward());
            operator.bSTART.whenReleased(() -> {
                Robot.hopper.stop();
                Robot.hopper.resetOverride();
            });

            // intake
            operator.bLB.whenPressed(Robot.intake::toggle);
            /*operator.bRB.whileHeld(() -> {
                Robot.hopper.update();
                Robot.intake.setSpeed(1);
            });
            operator.bRB.whenReleased(() -> {
                Robot.hopper.stop();
                Robot.intake.setSpeed(0);
            });*/

            Robot.hopper.update(Helper.analogToDigital(operator.getRT(), .1, .6));
            Robot.intake.setSpeed(Helper.analogToDigital(operator.getRT(), .1, 1));
            Robot.hopper.reverse(Helper.analogToDigital(operator.getLT(), .1, .6));
            Robot.intake.setSpeed(Helper.analogToDigital(operator.getLT(), .1, -1));

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

        updateSD();
    }

    public static void updateSD() {
        SmartDashboard.putString("WoF Color", Robot.wheel.toString());
        SmartDashboard.putBoolean("Left limit", Robot.turret.getLeftLimit()); // TODO: for testing
        SmartDashboard.putBoolean("Right limit", Robot.turret.getRightLimit()); // TODO: for testing
        SmartDashboard.putNumber("Turret pos", Robot.turret.getPracticeEncPosition()); // TODO: for testing
        SmartDashboard.putBoolean("Bot Sensor", Robot.hopper.getBotSensor());
        SmartDashboard.putBoolean("Mid Sensor", Robot.hopper.getMidLimit());
        SmartDashboard.putBoolean("Top Sensor", Robot.hopper.getTopLimit());
    }
}

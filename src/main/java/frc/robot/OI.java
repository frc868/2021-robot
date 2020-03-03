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

    public static void init() {
        Robot.shooter.init();
    }

    public static void update() {
        // HUGE MEGA TODO: figure out controls with driver and operator
        // GENERAL CONTROLS/CONTROL METHODS
        Robot.drivetrain.arcadeDrive(1);
        
        //TODO: change manual turret to joystick

        // DRIVER CONTROLS

        // OPERATOR CONTROLS

        // turret
        /*operator.bLB.whileHeld(() ->*/ Robot.turret.manualTurret();/*);*/
        operator.bLB.whenReleased(() -> {});

        // shoot
        operator.bA.whileHeld(() -> Robot.shooter.setSpeed(-0.6));
        operator.bA.whenReleased(() -> Robot.shooter.stop());
        operator.bSTART.whileHeld(() -> Robot.hopper.forward());
        operator.bSTART.whenReleased(() -> {
            Robot.hopper.stop();
            Robot.hopper.resetOverride();
        });

        // intake
        operator.bY.whenPressed(() -> Robot.intake.toggle());
        /*operator.bLB.whenReleased(() -> {});
        operator.bRB.whileHeld(() -> {
            Robot.hopper.update();
            Robot.intake.setSpeed(1);
        });
        operator.bRB.whenReleased(() -> {
            Robot.hopper.stop();
            Robot.intake.setSpeed(0);
        });*/

        
        //Robot.intake.setSpeed(Helper.analogToDigital(operator.getRT(), .1, 1) - Helper.analogToDigital(operator.getLT(), .1, 1));
        operator.bRT.whenPressed(() -> {
            Robot.hopper.update(.6);
            Robot.intake.setSpeed(1);
        });
        operator.bLT.whenPressed(() -> {
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
        operator.bB.whenReleased(() -> Robot.hopper.stop());

        // WOF
        operator.dN.whenPressed(() -> Robot.wheel.actuatorUp());
        operator.dN.whenReleased(() -> {});
        operator.dS.whenPressed(() -> Robot.wheel.actuatorDown());
        operator.dS.whenReleased(() -> {});

        // if it hasn't already been handled...
        driver.updateStates();
        operator.updateStates();

        updateSD();
    }

    public static void updateSD() {
        SmartDashboard.putString("WoF Color", Robot.wheel.toString());
        SmartDashboard.putNumber("Turret pos", Robot.turret.getPracticeEncPosition()); // TODO: for testing
        SmartDashboard.putBoolean("Bot Sensor", Robot.hopper.getBotSensor());
        SmartDashboard.putBoolean("Mid Sensor", Robot.hopper.getMidLimit());
        SmartDashboard.putBoolean("Top Sensor", Robot.hopper.getTopLimit());

        SmartDashboard.putNumber("Left trigger", operator.getLT());
        SmartDashboard.putNumber("Right trigger", operator.getRT());
    }
}

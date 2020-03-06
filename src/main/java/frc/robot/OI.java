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
        initSD();
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
        // operator.bLB.whenReleased(() -> {});
        operator.bRB.whileHeld(() -> Robot.turret.trackVision());
        operator.bRB.whenReleased(() -> Robot.turret.stop());

        // shoot
        operator.bA.whileHeld(() -> Robot.shooter.setSpeed(0.6));
        operator.bA.whenReleased(() -> Robot.shooter.stop());
        operator.bSTART.whileHeld(() -> {
            Robot.shooter.update();
            Robot.hopper.forward(Robot.shooter.atTarget());
        });
        operator.bSTART.whenReleased(() -> {
            Robot.shooter.stop();
            Robot.hopper.stop();
        });

        // intake
        operator.bY.whenPressed(() -> Robot.intake.toggle());
        operator.bY.whenReleased(() -> {});
        // operator.bRB.whileHeld(() -> {
        //     Robot.hopper.update();
        //     Robot.intake.setSpeed(1);
    
        // });
        // operator.bRB.whenReleased(() -> {
        //     Robot.hopper.stop();
        //     Robot.intake.setSpeed(0);
        // });
        
        Robot.intake.setSpeed(Helper.analogToDigital(operator.getRT(), .1, 1) - Helper.analogToDigital(operator.getLT(), .1, 1));
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
        operator.bB.whenReleased(() -> Robot.hopper.stop());

        // WOF
        operator.dN.whenPressed(() -> Robot.wheel.actuatorUp());
        operator.dN.whenReleased(() -> {});
        operator.dS.whenPressed(() -> Robot.wheel.actuatorDown());
        operator.dS.whenReleased(() -> {});

        driver.bRB.whileHeld(() -> Robot.turret.trackVision());
        driver.bRB.whenReleased(() -> Robot.turret.stop());

        // if it hasn't already been handled...
        driver.updateStates();
        operator.updateStates();

        updateSD();
    }

    public static void initSD() {
    }

    public static void updateSD() {
        SmartDashboard.putString("WoF Color", Robot.wheel.toString());
        SmartDashboard.putBoolean("Bot Sensor", Robot.hopper.getBotSensor());
        SmartDashboard.putBoolean("Mid Sensor", Robot.hopper.getMidLimit());
        SmartDashboard.putBoolean("Top Sensor", Robot.hopper.getTopLimit());
        SmartDashboard.putNumber("Hopper count", Robot.hopper.getBallCount());

        SmartDashboard.putBoolean("Turret left", Robot.turret.getLeftLimit());
        SmartDashboard.putBoolean("Turret right", Robot.turret.getRightLimit());

        SmartDashboard.putNumber("Left trigger", operator.getLT());
        SmartDashboard.putNumber("Right trigger", operator.getRT());

        SmartDashboard.putBoolean("At target", Robot.shooter.atTarget());
    }
}

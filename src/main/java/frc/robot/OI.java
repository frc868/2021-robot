package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helpers.ControllerWrapper;
import frc.robot.Robot;

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

        // DRIVER CONTROLS
        driver.bA.whenPressed(() -> Robot.shooter.setSpeed(0.8));
        driver.bA.whenReleased(() -> Robot.shooter.setSpeed(0));

        driver.bB.whenPressed(() -> Robot.intake.setSpeed(0.8));
        driver.bB.whenReleased(() -> Robot.intake.setSpeed(0));

        driver.bX.whileHeld(() -> Robot.turret.trackVision());
        driver.bX.whenReleased(() -> Robot.turret.stop());

        updateSD();
    }

    public static void updateSD() {
        SmartDashboard.putString("WoF Color", Robot.wheel.toString());
        SmartDashboard.putBoolean("Left limit", Robot.turret.getLeftLimit()); // TODO: for testing
        SmartDashboard.putBoolean("Right limit", Robot.turret.getRightLimit()); // TODO: for testing
        SmartDashboard.putNumber("Turret pos", Robot.turret.getPracticeEncPosition()); // TODO: for testing
        System.out.println(Robot.camera.toString());
    }
}

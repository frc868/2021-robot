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
        // Robot.drivetrain.arcadeDrive(0.5);

        // DRIVER CONTROLS

        driver.bB.whileHeld(() -> Robot.hopper.test());
        driver.bB.whenReleased(() -> Robot.hopper.stop());
        driver.bA.whileHeld(() -> Robot.hopper.shoot());
        driver.bA.whenReleased(() -> {
            Robot.hopper.stop();
            Robot.hopper.resetOverride();

        });
        driver.bRB.whileHeld(() -> {
            Robot.hopper.update();
            Robot.intake.setSpeed(1);
        });
        driver.bRB.whenReleased(() -> {
            Robot.hopper.stop();
            Robot.intake.setSpeed(0);
        });        
 
        updateSD();
    }

    public static void updateSD() {
        SmartDashboard.putString("WoF Color", Robot.wheel.toString());
        SmartDashboard.putString("IR LImit", Robot.hopper.toString());
    }
}

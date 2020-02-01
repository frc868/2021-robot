package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helpers.ControllerWrapper;

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
        driver.bA.whenPressed(() -> Robot.hopper.shoot());
        driver.bA.whenReleased(() -> Robot.hopper.resetOverride());
      
        Robot.turret.setSpeed(driver.getLX());
        updateSD();
    }

    public static void updateSD() {
        SmartDashboard.putString("Vision Data", Robot.camera.toString());
    }
}
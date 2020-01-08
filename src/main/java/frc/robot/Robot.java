/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WheelOfFortune;

/** 
 * This class is run automatically, and dictates what functions are run during each of these stages.
 * @author dri
 */
public class Robot extends TimedRobot {
    public static Camera camera = Camera.getInstance();
    public static Climber climber = Climber.getInstance();
    public static Drivetrain drivetrain = Drivetrain.getInstance();
    public static Intake intake = Intake.getInstance();
    public static Shooter shooter = Shooter.getInstance();
    public static WheelOfFortune wheel = WheelOfFortune.getInstance();
    
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        OI.init();
    }

    @Override
    public void teleopPeriodic() {
        OI.update();
        Scheduler.getInstance().run();
    }

    @Override
    public void testInit() {
        OI.init();
    }

    @Override
    public void testPeriodic() {
        OI.update();
        Scheduler.getInstance().run();
    }
}

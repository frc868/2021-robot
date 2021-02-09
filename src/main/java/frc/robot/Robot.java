/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.autonomous.AutonHelper;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/** 
 * This class is run automatically, and dictates what functions are run during each of these stages.
 * @author dri
 */
public class Robot extends TimedRobot {
    public static AutonHelper auton = AutonHelper.getInstance();
    public static Camera camera = Camera.getInstance();
    public static Climber climber = Climber.getInstance();
    public static Compressor compressor = new Compressor();
    public static Drivetrain drivetrain = Drivetrain.getInstance();
    public static Gyro gyro = Gyro.getInstance();
    public static Intake intake = Intake.getInstance();
    public static LED leds = LED.getInstance();
    public static Hopper hopper = Hopper.getInstance(true);
    public static Shooter shooter = Shooter.getInstance();
    public static Turret turret = Turret.getInstance(true);

    @Override
    public void disabledInit() {
        // auton.initSD();
        // auton.resetSelectedPath();
    }

    @Override
    public void robotInit() {
        shooter.init();
        turret.resetEncoders();
    }

    @Override
    public void robotPeriodic() {
        OI.updateSD();
        camera.update();
        leds.colorInventory();
        // System.out.println(camera.toString());
    }

    @Override
    public void autonomousInit() {
        // drivetrain.resetInitialDistance();
        // drivetrain.resetEncoderPositions();
        // gyro.reset();
    }

    @Override
    public void autonomousPeriodic() {
        OI.updateSD();

        auton.runSelectedPath();
        turret.trackVision();
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        OI.init();
    }

    @Override
    public void teleopPeriodic() {
        OI.updateSD();

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

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.helpers.Helper;

/**
 * The turret being used to possibly position the shooter. This is nothing more 
 * than a prototype as it currently stands, and may need major refactoring or 
 * complete deletion for the final robot.
 * 
 * @author hrl, dri
 */
public class Turret {
    private static Turret instance;

    private WPI_TalonSRX motor;

    private PIDController pid;

    private DigitalInput leftLimit;
    private DigitalInput rightLimit;

    // for tracking target, TODO; tune
    private final double kP = 0.01, kI = 0, kD = 0;
    private final double MAX_POS = 30; // maximum angle for x-position
    
    private double zeroPos;
    private double zeroAngle;
    
    private Turret() {
        motor = new WPI_TalonSRX(RobotMap.Turret.MOTOR);        
        motor.setInverted(RobotMap.Turret.MOTOR_IS_INVERTED);
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); // TODO: untested

        leftLimit = new DigitalInput(RobotMap.Turret.Limits.LEFT_PORT);
        rightLimit = new DigitalInput(RobotMap.Turret.Limits.RIGHT_PORT);

        pid = new PIDController(kP, kI, kD);

        zeroPos = motor.getSensorCollection().getQuadraturePosition();
        zeroAngle = Robot.gyro.getAngle();
    }

    /**
     * Returns a singular instance of the Turret subsystem.
     */
    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    /**
     * Sets the turret speed.
     * @param speed the speed to be set between -1 and 1
     */
    public void setSpeed(double speed) {
        if (leftLimit.get() || rightLimit.get()) { // TODO: untested limit switch states
            stop();
        }
        motor.set(Helper.boundValue(speed));
    }

    /**
     * Gets the current turret speed.
     * @return the current turret speed between -1 and 1
     */
    public double getSpeed() {
        return motor.get();
    }

    /**
     * Gets the current turret position
     * @return the current quadrature encoder position
     */
    public double getPosition() {
        return motor.getSensorCollection().getQuadraturePosition();
    }

    /**
     * Stops the turret.
     */
    public void stop() {
        this.setSpeed(0);
    }

    /**
     * Tracks the current shooting target.
     */
    public void trackVision() {
        if (Robot.camera.hasTarget() && (Math.abs(Robot.camera.getPosition()) < MAX_POS)) {
            this.setSpeed(pid.calculate(Robot.camera.getPosition(), 0));
        } else {
            this.stop();
        }

        SmartDashboard.putData(this.pid);
    }

    /**
     * Moves turret to safe position to avoid hitting wheel of fortune or climber arms 
     * when they go up
     */
    public void safeZone() {
        if (getPosition() < RobotMap.Turret.Setpoints.SAFE_POSITION && getPosition() > RobotMap.Turret.Setpoints.DEADZONE_LEFT) {
            setSpeed(0.2);
        }
        else if (getPosition() > RobotMap.Turret.Setpoints.SAFE_POSITION && getPosition() < RobotMap.Turret.Setpoints.DEADZONE_RIGHT) {
            setSpeed(-0.2);
        }
        else {
            stop();
        }
    }

    /**
     * Lowest level of turret control; uses gyroscope angle to point turret towards the goal
     */
    public void trackGoal() {
        double currentAngle = Robot.gyro.getAngle();
        double angleError = currentAngle - zeroAngle;

        
    }
}
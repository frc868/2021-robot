/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
 * @author hrl
 */
public class Turret {
    private static Turret instance;

    private WPI_TalonSRX motor;

    private PIDController pid;

    // for tracking target, TODO; tune
    private final double kP = 0.012, kI = 0, kD = 0.0001;
    private final double MAX_POS = 30; // maximum angle for x-position
    
    private Turret() {
        motor = new WPI_TalonSRX(RobotMap.Turret.MOTOR);        
        motor.setInverted(RobotMap.Turret.MOTOR_IS_INVERTED);
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); // TODO: untested

        pid = new PIDController(kP, kI, kD);
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
     * Stops the turret.
     */
    public void stop() {
        this.setSpeed(0);
    }

    /**
     * Tracks the current shooting target.
     */
    public void track() {
        if (Robot.camera.hasTarget() && (Math.abs(Robot.camera.getPosition()) < MAX_POS)) {
            this.setSpeed(pid.calculate(Robot.camera.getPosition(), 0));
        } else {
            this.stop();
        }

        SmartDashboard.putData(this.pid);
    }
}

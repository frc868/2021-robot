/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * This is the code for the shooter. It initializes motor controllers and has
 * methods for various functions of the shooter. It also uses PID control for
 * maintaining optimal velocity.
 * 
 * @author ama, gjs, hrl
 */
public class Shooter {
    private static Shooter instance = null;

    private CANSparkMax primary;
    private CANSparkMax secondary;

    private CANPIDController pid;

    private double kP, kI, kD, kFF, kIa, setpoint;

    /**
     * The shooter subsystem consists of the two-neo shooter mounted on the robot's turret.
     * It is controlled with REV's PID Controller on the SparkMAXes.
     * 
     * @author dri
     */
    private Shooter() {
        primary = new CANSparkMax(RobotMap.Shooter.PRIMARY, MotorType.kBrushless);
        secondary = new CANSparkMax(RobotMap.Shooter.SECONDARY, MotorType.kBrushless);

        primary.setInverted(RobotMap.Shooter.PRIMARY_IS_INVERTED);
        secondary.follow(primary, RobotMap.Shooter.SECONDARY_IS_OPPOSITE);
        // Secondary motor is always inverted relative to primary

        primary.restoreFactoryDefaults();
        secondary.restoreFactoryDefaults();
        
        pid = primary.getPIDController();

        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);
        SmartDashboard.putNumber("kFF", 0);
        SmartDashboard.putNumber("kIa", 0);
        SmartDashboard.putNumber("Setpoint", 0);
    }

    /**
     * Returns the instance of the Shooter class
     * @return instance of shooter
     */
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    /**
     * Sets the PID gains and setpoint for the PID controller.
     */
    public void init() {
        kP = SmartDashboard.getNumber("kP", 0);
        kI = SmartDashboard.getNumber("kI", 0);
        kD = SmartDashboard.getNumber("kD", 0);
        kFF = SmartDashboard.getNumber("kFF", 0);
        kIa = SmartDashboard.getNumber("kIa", 0);
        setpoint = SmartDashboard.getNumber("Setpoint", RobotMap.Shooter.SHOOTER_DEFAULT_SPEED);

        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setFF(kFF);

        if (this.kI == 0) {
            pid.setIMaxAccum(0, 0);
        } else {
            pid.setIMaxAccum(kIa, 0);
        }
        
        pid.setOutputRange(0, 1);
    }

    /**
     * Sets the output of the PID loop to the primary motor. 
     */
    public void update() {
        pid.setReference(setpoint, ControlType.kVelocity);
        SmartDashboard.putNumber("Output", primary.getEncoder().getVelocity());
    }

    /**
     * Manually sets the speed of the motors.
     * @param speed the speed from -1 to 1
     */
    public void setSpeed(double speed) {
        primary.set(speed);
    }

    /**
     * Stops the shooter.
     */
    public void stop() {
        primary.stopMotor();
        secondary.stopMotor();
    }
}

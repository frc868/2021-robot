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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Robot;
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

    private double kP = 0, kI = 0, kD = 0;
    private double kIa = 0, kIz = 0, kFF = 0;
    private double kMaxOutput = 1;
    private double kMinOutput = -1;

    private double setpoint = 0;

    private Shooter() {
        // initialise SMaxes
        primary = new CANSparkMax(RobotMap.Shooter.PRIMARY, MotorType.kBrushless);
        secondary = new CANSparkMax(RobotMap.Shooter.SECONDARY, MotorType.kBrushless);
        primary.setInverted(RobotMap.Shooter.PRIMARY_IS_INVERTED);
        secondary.setInverted(RobotMap.Shooter.SECONDARY_IS_INVERTED);

        secondary.follow(primary);
        pid = primary.getPIDController();

        // reset the motor controllers
        primary.restoreFactoryDefaults();
        secondary.restoreFactoryDefaults();

        // don't bother braking, it's a *shooter*
        primary.setIdleMode(IdleMode.kCoast);
        secondary.setIdleMode(IdleMode.kCoast);

        // by default
        this.disable();
    }

    /**
     * Returns a singleton instance of the shooter subsystem.
     */
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    /**
     * Enables the Spark MAX PID controller. This sucks, man.
     */
    public void enable() {
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setIAccum(kIa);
        pid.setIZone(kIz);
        pid.setFF(kFF);
        pid.setOutputRange(kMaxOutput, kMinOutput);
    }

    /**
     * Disables the Spark MAX PID controller. I actively hate this so much.
     */
    public void disable() {
        pid.setP(0);
        pid.setI(0);
        pid.setD(0);
        pid.setIAccum(0);
        pid.setIZone(0);
        pid.setFF(0);
        pid.setOutputRange(0, 0);
    }

    /**
     * Sets the setpoint of the shooter's PID.
     * @param setpoint the setpoint in RPM
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        pid.setReference(this.setpoint, ControlType.kVelocity);
    }

    /**
     * sets the speed of the shooter manually
     * @param speed the speed to set from -1 to 1
     */
    public void setSpeed(double speed) {
        this.disable();
        primary.set(speed);
    }

    /**
     * Stops the motor manually.
     */
    public void stop() {
        this.disable();
        primary.stopMotor();
    }

    /**
     * Initializes the SmartDashboard data for the shooter subsystem.
     */
    public void initSD() {
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Accum", kIa);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Setpoint", setpoint);
    }

    /**
     * Updates both the state of the SmartDashboard and the PID's variables
     * accordingly.
     */
    public void updateSD() {
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double ia = SmartDashboard.getNumber("I Accum", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        if (i == 0) {
            pid.setIAccum(0);
        }

        kP = p;
        kI = i;
        kD = d;
        kIa = ia;
        kIz = iz;
        kFF = ff;
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            kMinOutput = min;
            kMaxOutput = max;
        }
    }

    /**
     * shoots until all balls are cleared from the hopper.
     * useful in autonomous.
     * TODO: this will be PID controlled!
     * @param pwr the power to run the shooter at; will be obsoleted by PID
     * @author hrl
     */
    public void shootUntilClear(double pwr) {
        if (Robot.hopper.getBallCount() > 0) {
            Robot.hopper.shoot();
            this.setSpeed(pwr);
        }
    }
}

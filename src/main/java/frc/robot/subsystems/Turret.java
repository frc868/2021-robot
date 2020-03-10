/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.OI;
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

    private Encoder compEncoder;
    private WPI_TalonSRX motor;

    private PIDController pidVision;
    private PIDController pid;
    private DigitalInput leftLimit;
    private DigitalInput rightLimit;

    // for tracking target with vision
    private double kPv, kIv, kDv;
    // for goal-centric gyro-based positioning
    private double kP, kI, kD;
    private final double MAX_POS = 30; // maximum angle for x-position

    private boolean isCompBot = true;
    
    private Turret(boolean compBot) {
        this.isCompBot = compBot;
        
        motor = new WPI_TalonSRX(RobotMap.Turret.MOTOR);
        motor.setNeutralMode(NeutralMode.Brake);

        if (compBot) {
            leftLimit = new DigitalInput(RobotMap.Turret.CompBot.Limits.LEFT_PORT);
            rightLimit = new DigitalInput(RobotMap.Turret.CompBot.Limits.RIGHT_PORT);

            motor.setInverted(RobotMap.Turret.CompBot.MOTOR_IS_INVERTED);

            compEncoder = new Encoder(RobotMap.Turret.CompBot.ENCODER_1, RobotMap.Turret.CompBot.ENCODER_2);

            kPv = RobotMap.Turret.CompBot.PID.kPv;
            kIv = RobotMap.Turret.CompBot.PID.kIv;
            kDv = RobotMap.Turret.CompBot.PID.kDv;

            kP = RobotMap.Turret.CompBot.PID.kP;
            kI = RobotMap.Turret.CompBot.PID.kI;
            kD = RobotMap.Turret.CompBot.PID.kD;
        }

        else {
            leftLimit = new DigitalInput(RobotMap.Turret.PracticeBot.Limits.LEFT_PORT);
            rightLimit = new DigitalInput(RobotMap.Turret.PracticeBot.Limits.RIGHT_PORT);

            motor.setInverted(RobotMap.Turret.PracticeBot.MOTOR_IS_INVERTED);

            kPv = RobotMap.Turret.PracticeBot.PID.kPv;
            kIv = RobotMap.Turret.PracticeBot.PID.kIv;
            kDv = RobotMap.Turret.PracticeBot.PID.kDv;

            kP = RobotMap.Turret.PracticeBot.PID.kP;
            kI = RobotMap.Turret.PracticeBot.PID.kI;
            kD = RobotMap.Turret.PracticeBot.PID.kD;
        }

        pidVision = new PIDController(kPv, kIv, kDv);
        pid = new PIDController(kP, kI, kD);
    }

    /**
     * Returns a singular instance of the Turret subsystem.
     */
    public static Turret getInstance(boolean compBot) {
        if (instance == null) {
            instance = new Turret(compBot);
        }
        return instance;
    }

    /**
     * Sets the turret speed.
     * @param speed the speed to be set between -1 and 1
     */
    public void setSpeed(double speed) {
        if (isCompBot) {
            if (leftLimit.get() == RobotMap.Turret.CompBot.Limits.LIMIT_TRIGGERED) {
                speed = Helper.boundValue(speed, 0, 1);
            }
            if (rightLimit.get() == RobotMap.Turret.CompBot.Limits.LIMIT_TRIGGERED) {
                speed = Helper.boundValue(speed, -1, 0);
            }
        }
        else {
            if (leftLimit.get() == RobotMap.Turret.PracticeBot.Limits.LIMIT_TRIGGERED) {
                speed = Helper.boundValue(speed, 0, 1);
            }
            if (rightLimit.get() == RobotMap.Turret.PracticeBot.Limits.LIMIT_TRIGGERED) {
                speed = Helper.boundValue(speed, -1, 0);
            }
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
    public double getPracticeEncPosition() {
        return motor.getSensorCollection().getQuadraturePosition();
    }

    /**
     * Stops the turret.
     */
    public void stop() {
        motor.set(0);
    }

    /**
     * Tracks the current shooting target.
     */
    public void trackVision() {
        double visionOffset = Robot.camera.getCalculatedDistance() * RobotMap.Turret.CompBot.PID.VISION_OFFSET_SCALE;
        if (Robot.camera.hasTarget() && (Math.abs(Robot.camera.getPosition()) < MAX_POS)) {
            double pidOutput = pidVision.calculate(Robot.camera.getPosition(), visionOffset);
            this.setSpeed(Helper.boundValue(pidOutput, -0.75, 0.75));
        } else {
            this.stop();
        }
    }

    /**
     * Moves turret to safe position to avoid hitting wheel of fortune or climber arms 
     * when they go up
     */
    public void safeZone() {
        if (this.isCompBot) {
            setPosition(RobotMap.Turret.CompBot.Setpoints.SAFE_POSITION);
        }
        else {
            setPosition(RobotMap.Turret.PracticeBot.Setpoints.SAFE_POSITION);
        }
    }

    /**
     * Allows manual control of the turret from operator controller triggers.
     * Limits speed to 0.25 at most. <i>Do not change this.</i>
     */
    public void manualTurret() {
        setSpeed(0.4*(OI.operator.getLX()));
    }

    /**
     * Allows turret to go to setpoints in its rotation, controlled with PID loop and encoder.
     * @param position desired position
     * @param compBot whether method is called for comp or practice bot
     */
    public void setPosition(double position) {
        double pidOutput;
        if (this.isCompBot) {
            pidOutput = pid.calculate(getCompEncPosition(), position);
        }
        else {
            pidOutput = pid.calculate(getPracticeEncPosition(), position);
        }
        this.setSpeed(pidOutput);
    }

    public double getAngle() {
        return motor.getSensorCollection().getQuadraturePosition();
    }

    @Override
    public String toString() {
        return "" + getAngle();
    }

    /**
     * Returns the current position of the encoder.
     * TODO: this should switch based on the boolean in the constructor
     */
    public double getCompEncPosition() {
        return compEncoder.getDistance();
    }

    public boolean getLeftLimit() {
        return leftLimit.get();
    }

    public boolean getRightLimit() {
        return rightLimit.get();
    }

    /**
     * Resets the encoders.
     */
    public void resetEncoders() {
        if (this.isCompBot) {
            compEncoder.reset();
        }
        else {
            motor.getSensorCollection().setQuadraturePosition(0, 0);
        }
    }
}

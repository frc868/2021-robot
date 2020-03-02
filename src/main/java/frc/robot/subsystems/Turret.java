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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
    private double zeroPos;
    private double zeroAngle;
    
    private Turret(boolean compBot) {
        motor = new WPI_TalonSRX(RobotMap.Turret.MOTOR);

        if (compBot) {
            leftLimit = new DigitalInput(RobotMap.Turret.CompBot.Limits.LEFT_PORT);
            rightLimit = new DigitalInput(RobotMap.Turret.CompBot.Limits.RIGHT_PORT);

            motor.setInverted(RobotMap.Turret.CompBot.MOTOR_IS_INVERTED);

            compEncoder = new Encoder(RobotMap.Turret.CompBot.ENCODER_1, RobotMap.Turret.CompBot.ENCODER_2);

            zeroPos = getCompEncPosition();

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

            zeroPos = motor.getSensorCollection().getQuadraturePosition();

            kPv = RobotMap.Turret.PracticeBot.PID.kPv;
            kIv = RobotMap.Turret.PracticeBot.PID.kIv;
            kDv = RobotMap.Turret.PracticeBot.PID.kDv;

            kP = RobotMap.Turret.PracticeBot.PID.kP;
            kI = RobotMap.Turret.PracticeBot.PID.kI;
            kD = RobotMap.Turret.PracticeBot.PID.kD;
        }
        

        resetEncoders(compBot);

        pidVision = new PIDController(kPv, kIv, kDv);
        pid = new PIDController(kP, kI, kD);

        zeroAngle = Robot.gyro.getAngle();
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
        if (leftLimit.get() == RobotMap.Turret.PracticeBot.Limits.LIMIT_TRIGGERED) { // TODO: add modularity
            speed = Helper.boundValue(speed, 0, 1);
        }
        if (rightLimit.get() == RobotMap.Turret.PracticeBot.Limits.LIMIT_TRIGGERED) {
            speed = Helper.boundValue(speed, -1, 0);
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
        if (Robot.camera.hasTarget() && (Math.abs(Robot.camera.getPosition()) < MAX_POS)) {
            double pidOutput = pidVision.calculate(Robot.camera.getPosition(), 0);
            this.setSpeed(pidOutput);
            SmartDashboard.putNumber("Camera pos", Robot.camera.getPosition());
            SmartDashboard.putNumber("PID output", pidOutput);
            System.out.println(pidOutput);
        } else {
            this.stop();
        }
        SmartDashboard.putData(this.pidVision);
    }

    /**
     * Moves turret to safe position to avoid hitting wheel of fortune or climber arms 
     * when they go up
     */
    public void safeZone(boolean compBot) {
        if (compBot) {
            if (getCompEncPosition() < RobotMap.Turret.CompBot.Setpoints.SAFE_POSITION &&
                getCompEncPosition() > RobotMap.Turret.CompBot.Setpoints.DEADZONE_LEFT) {

                setSpeed(0.2);
            }
            else if (getCompEncPosition() > RobotMap.Turret.CompBot.Setpoints.SAFE_POSITION &&
                     getCompEncPosition() < RobotMap.Turret.CompBot.Setpoints.DEADZONE_RIGHT) {
                    
                setSpeed(-0.2);
            }
            else {
                stop();
            }
        }
        else {
            if (getPracticeEncPosition() < RobotMap.Turret.PracticeBot.Setpoints.SAFE_POSITION &&
                getPracticeEncPosition() > RobotMap.Turret.PracticeBot.Setpoints.DEADZONE_LEFT) {

                setSpeed(0.2);
            }
            else if (getPracticeEncPosition() > RobotMap.Turret.PracticeBot.Setpoints.SAFE_POSITION &&
                     getPracticeEncPosition() < RobotMap.Turret.PracticeBot.Setpoints.DEADZONE_RIGHT) {

                setSpeed(-0.2);
            }
            else {
                stop();
            }
        }
    }

    /**
     * Lowest level of turret control; uses gyroscope angle to point turret towards the goal
     */
    public void trackGoal(boolean compBot) {
        double currentAngle = Robot.gyro.getAngle();
        double angleError = currentAngle - zeroAngle;

        if (compBot) {
            double scaledError = angleError * RobotMap.Turret.CompBot.PID.GYRO_TO_ENCODER;
            setPosition(zeroPos - scaledError, compBot);
        }
        else {
            double scaledError = angleError * RobotMap.Turret.PracticeBot.PID.GYRO_TO_ENCODER;
            setPosition(zeroPos - scaledError, compBot);
        }        
    }

    /**
     * Allows manual control of the turret from operator controller triggers.
     * Limits speed to 0.3 at most.
     */
    public void manualTurret() {
        setSpeed(0.3*(OI.driver.getLT() - OI.driver.getRT()));
    }

    /**
     * Allows turret to go to setpoints in its rotation, controlled with PID loop and encoder.
     * @param position desired position
     * @param compBot whether method is called for comp or practice bot
     */
    public void setPosition(double position, boolean compBot) {
        double pidOutput;
        if (compBot) {
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

    public double getCompEncPosition() {
        return compEncoder.getDistance();
    }

    public boolean getLeftLimit(boolean compBot) {
        return leftLimit.get();
    }

    public boolean getRightLimit(boolean compBot) {
        return rightLimit.get();
    }

    public void resetEncoders(boolean compBot) {
        if (compBot) {
            compEncoder.reset();
        }
        else {
            motor.getSensorCollection().setQuadraturePosition(0, 0);
        }
    }

}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.helpers.Helper;

/** 
 * This is the code for the shooter. It initializes motor controllers and has methods
 * for various functions of the shooter. It also uses PID control for maintaining optimal velocity.
 * Is probably unfinished and subject to massive revision
 * 
 * @author ama
 */
 public class Shooter {
    private static Shooter instance = null;

    private CANSparkMax motor;
    private PIDController pid;

    // Setpoints and Potentiometer limits
    // private final double LOWER = 0.85, MIDDLE = 0.831, UPPER = 0.726;
    private static final double kP = 0, kI = 0, kD = 0; // TODO: untested

    private Shooter() {
        motor = new CANSparkMax(RobotMap.Shooter.MOTOR, MotorType.kBrushless);
        CANError err =  motor.restoreFactoryDefaults(); // Reset the Spark Max to factory defaults to avoid sticky values from previous deploys
        if(err != CANError.kOk) {
            System.out.println("Error resetting Spark Max to factory defaults: " + err.toString()); // TODO: untested
        }
        
        motor.setInverted(RobotMap.Shooter.MOTOR_IS_INVERTED); // TODO: untested

        pid = new PIDController(kP, kI, kD);
    }

    /**
     * returns a singleton instance of the Shooter subsystem
     */
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    /**
     * sets the speed of the shooter manually
     * @param speed the speed to set from -1 to 1
     */
    public void setSpeed(double speed) {
        motor.set(pid.calculate(Helper.boundValue(motor.getEncoder().getVelocity(), -1, 1)));
    }

    /**
     * stops the shooter.
     */
    public void stop() {
        motor.stopMotor();
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

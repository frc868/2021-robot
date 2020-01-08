/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
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
    private static final double kP = 0, kI = 0, kD = 0; //TODO: untested

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        motor = new CANSparkMax(RobotMap.Shooter.MOTOR,MotorType.kBrushless);
        motor.setInverted(true); //TODO: untested

        pid = new PIDController(kP, kI, kD);
    }

    public void setSpeed(double speed) {
            motor.set(pid.calculate(Helper.boundValue(motor.getEncoder().getVelocity(), -1, 1)));
    }

    public void stop() {
        motor.stopMotor();
    }
}

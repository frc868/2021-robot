/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;
import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.Robot;

/**
 * This class rotates the robot to the specified angle (in degrees) using a Gyro sensor.
 * It uses a PID controller to accomplish this behavior.
 * 
 * @author jw
 */
public class TurnToAngleGyro {
    private static final double kP = 0, kI = 0, kD = 0; // TODO: untested
    private final double tolerance = 1; // TODO: untested
    private PIDController pid;

    private int angle; // setpoint

    /**
     * Creates a new TurnToAngleGyro object, to be run with TurnToAngleGyro.run().
     * @param angle the angle to turn to, from -180 to +180
     */
    public TurnToAngleGyro(int angle) {
        // initialize sources
        pid = new PIDController(kP, kI, kD);
        this.angle = angle;
    }
   
    /**
     * Turns the robot to the specified angle (in degrees).
     */
    public void run() {
        double currentAngle = Robot.gyro.getAngle(); // relative, without gyro reset

        // TODO: check + vs - turn direction
        // if we want to turn "positive" AKA "left"
        if (angle > 0) {
            //set angle to desired angle
            angle += currentAngle;

            // navX reads from -180* to 180* 
            // if desired angle exceeds gyro readings
            if (angle > 180) {
                angle -= 360;
            }

            // run until reaches setpoint
            while (currentAngle < angle - tolerance || currentAngle > angle + tolerance) {
                // calculate speed based on distance from current angle and desired angle
                double speed = pid.calculate(currentAngle, angle);

                // set motor
                Robot.drivetrain.setSpeed(speed, -speed);
            }
        }
        else { // else we want to turn "negative" (aka right)
            // set angle to desired angle
            angle -= currentAngle;

            // navX reads from -180* to 180* 
            // if desired angle exceeds gyro readings
            if (angle < -180) {
                angle += 360;
            }

            // run until reaches setpoint
            while (currentAngle < angle - tolerance || currentAngle > angle + tolerance) {
                // calculate speed based on distance from current angle and desired angle
                double speed = pid.calculate(currentAngle, angle);

                Robot.drivetrain.setSpeed(-speed, speed);
            }
        }

        Robot.drivetrain.setSpeed(0, 0);
    }
}
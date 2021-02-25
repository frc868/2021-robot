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
 * @author jw, gjs
 */
public class TurnToAngleGyro {
    private static final double kP = 0.0039, kI = 0, kD = 0; // TODO: untested
    private final double tolerance = 1; // TODO: untested
    private PIDController pid;
    private static TurnToAngleGyro instance;

    public static TurnToAngleGyro getInstance(){
        if(instance == null){
            return new TurnToAngleGyro();
        }
        return instance;
    }

    private double initialAngle;

    /**
     * Creates a new TurnToAngleGyro object, to be run with TurnToAngleGyro.run().
     * @param angle the angle to turn to, from -180 to +180
     */
    private TurnToAngleGyro() {
        // initialize sources
        pid = new PIDController(kP, kI, kD);

        this.initialAngle = Robot.gyro.getAngle();
    }

    /**
     * Resets the initial angle to wherever the robot currently is.
     */
    public void reset() {
        // this.initialAngle = Robot.gyro.getAngle();
        Robot.gyro.reset();
        System.out.println(Robot.gyro.getAngle());
    }

    /**
     * Turns the robot to the specified angle (in degrees).
     */
    public void run(double angle, double maxSpeed) {
        double targetSpeed = 0;
        Robot.gyro.reset();
        double currentAngle = Robot.gyro.getAngle();
        System.out.println(currentAngle);

        while (currentAngle != angle) {
            // turns the robot to the right
            if (angle > 0.0 && angle <= 180.0) {
                while (currentAngle < angle / 2) {
                    targetSpeed = 0.1 + (currentAngle / (angle / 2));
                    Robot.drivetrain.setSpeed(maxSpeed * targetSpeed, -maxSpeed * targetSpeed);
                    currentAngle = Robot.gyro.getAngle();
                }
                
                System.out.println("Midpoint reached: " + currentAngle);
                // System.out.println();
        
                while (currentAngle >= angle / 2) {
                    targetSpeed = 1 - ((currentAngle-angle/2) / (angle / 2));
                    Robot.drivetrain.setSpeed(maxSpeed * targetSpeed, -maxSpeed * targetSpeed);
                    currentAngle = Robot.gyro.getAngle();
                    System.out.println(currentAngle);
                }
            }

            // turns the robot to the left
            if (angle < 0.0 && angle >= -180.0) {
                while (currentAngle < angle / 2) {
                    targetSpeed = 0.1 + (currentAngle / (angle / 2));
                    Robot.drivetrain.setSpeed(-maxSpeed * targetSpeed, maxSpeed * targetSpeed);
                    currentAngle = Robot.gyro.getAngle();
                }
                
                System.out.println("Midpoint reached: " + currentAngle);
                // System.out.println();
        
                while (currentAngle >= angle / 2) {
                    targetSpeed = 1 - ((currentAngle-angle/2) / (angle / 2));
                    Robot.drivetrain.setSpeed(-maxSpeed * targetSpeed, maxSpeed * targetSpeed);
                    currentAngle = Robot.gyro.getAngle();
                    System.out.println(currentAngle);
                }
            }
        }
    }
}

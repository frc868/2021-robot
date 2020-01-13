/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.helpers.Helper;

/**
 * Controls the Limelight camera connected to the robot over NetworkTables.
 * 
 * @author hrl
 */
public class Camera {
    private static Camera instance = null;

    // represents the current mode of the camera
    public static enum CameraPipeline {
        SHOOTER, BALL;
    }

    private NetworkTable table;
    private NetworkTableEntry tEnabled, tArea, tXpos, tAngle, tPipeline;
    private PIDController pid;

    // for tracking target, TODO; tune
    private final double kP = 0.01, kI = 0, kD = 0;
    private final double MAX_POS = 30; // maximum angle for x-position

    // for tracking ball, TODO: tune
    private final double kDist = 0;
    private final double kPos = 0;
    private final double kArea = 0;

    private Camera() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.update(); // initial run

        pid = new PIDController(kP, kI, kD);
    }

    /**
     * Get the singleton instance of the Camera class.
     */
    public static Camera getInstance() {
        if (instance == null) {
            instance = new Camera();
        }
        return instance;
    }

    /**
     * Gets the current camera pipeline.
     * @return the current pipeline as a double
     */
    public double getMode() {
        return tPipeline.getDouble(0.0);
    }

    /**
     * Switches the mode of the Limelight from shooter target to ball (and vice versa).
     */
    public void toggleMode() {
        if (this.getMode() == RobotMap.Camera.SHOOTER_PIPELINE) {
            tPipeline.setDouble(RobotMap.Camera.BALL_PIPELINE);
        } else if (this.getMode() == RobotMap.Camera.BALL_PIPELINE) {
            tPipeline.setDouble(RobotMap.Camera.SHOOTER_PIPELINE);
        }
    }

    /**
     * A standard method for updating a subsystem or sensor.
     * Call in robotPeriodic().
     * 
     * This updates the various state variables used for followVision.
     */
    public void update() {
        tEnabled = table.getEntry("tv"); // target enabled?
        tArea = table.getEntry("ta"); // target area
        tXpos = table.getEntry("tx"); // target x-position (y unused)
        tAngle = table.getEntry("ts"); // target angle/"skew"
        tPipeline = table.getEntry("pipeline"); // current pipeline, for mode switching
    }

    /**
     * Determines whether the target has been found.
     * @return boolean representing target detection status
     */
    public boolean hasTarget() {
        return tEnabled.getDouble(0.0) == 1;
    }

    /**
     * Returns the area of the target, if detected.
     * @return 0.0 if no target, area otherwise
     */
    public double getArea() {
        return tArea.getDouble(0.0);
    }

    /**
     * Get the x-position of the target, if detected.
     * @return 0.0 if no target, x-position otherwise
     */
    public double getPosition() {
        return tXpos.getDouble(0.0);
    }

    /**
     * Get the angle (with respect to the horizontal axis) of the target, if detected.
     * @return 0.0 if no target, x-angle otherwise
     */
    public double getAngle() {
        double angle = tAngle.getDouble(0.0);
        if (angle < -45) {
            angle += 90;
        }
        return angle;
    }

    /**
     * Tracks the current shooting target. Only to be used in shooter mode.
     */
    public void trackTarget() {
        if (this.hasTarget() && (Math.abs(this.getPosition()) < MAX_POS)) {
            Robot.turret.setSpeed(pid.calculate(this.getPosition(), 0));
        } else {
            Robot.turret.stop();
        }

        SmartDashboard.putData(this.pid);
    }

    /**
     * Tracks the current ball. Only to be used in ball mode.
     * An adaptation of 2019-Robot-B3's followVision() method.
     * (And when I mean adaptation, I mean literally the same method.)
     */
    public void trackBall() {
        if (this.hasTarget()) {
            double area = this.getArea();
            double posError = this.getPosition(); // how far we are from the target
            // the target value we are going to
            double posValue = posError * kPos * Math.sqrt(Helper.boundValue((area * kArea), 0, 1));

            // powers to set drivetrain to
            double left = Helper.boundValue((1/Math.sqrt(area)) * kDist + posValue);
            double right = Helper.boundValue((1/Math.sqrt(area)) * kDist + posValue);

            Robot.drivetrain.setSpeed(left, right);
        }
    }

    /**
     * Tracks in a general fashion.
     */
    public void track() {
        if (this.getMode() == RobotMap.Camera.SHOOTER_PIPELINE) {
            this.trackTarget();
        } else if (this.getMode() == RobotMap.Camera.BALL_PIPELINE) {
            this.trackBall();
        }
    }

    /**
     * Print the position, angle, and target status, for use on the SmartDashboard.
     * @return "Position,Angle,TargetFound"
     */
    @Override
    public String toString() {
        String toString = "" + this.getPosition() + "," + this.getAngle() + "," + this.hasTarget();
        return toString;
    }
}

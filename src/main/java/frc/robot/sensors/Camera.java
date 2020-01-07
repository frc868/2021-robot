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
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.helpers.Helper;

/**
 * Controls the Limelight camera connected to the robot over NetworkTables.
 * 
 * @author hrl
 */
public class Camera {
    private static Camera instance = null; // the instance to be used for getInstance()

    private NetworkTable table;
    private NetworkTableEntry tv, ta, tx, ts;

    // TODO: tune these! copied from 2019-Robot
    private static final double kDist = 0.18;
    private static final double kPos = 0.008;
    private static final double kArea = 0.1;

    private Camera() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.update(); // initial run
    }

    /**
     * Gets the singleton instance of the Camera class.
     */
    public static Camera getInstance() {
        if (instance == null) {
            instance = new Camera();
        }
        return instance;
    }

    /**
     * A standard method for updating a subsystem or sensor.
     * Call in robotPeriodic().
     * 
     * This updates the various state variables used for followVision.
     */
    public void update() {
        tv = table.getEntry("tv"); // target enabled?
        ta = table.getEntry("ta"); // target area
        tx = table.getEntry("tx"); // target x-position (y unused)
        ts = table.getEntry("ts"); // target angle
    }

    /**
     * Determines whether the target has been found.
     * @return boolean representing target detection status
     */
    public boolean hasTarget() {
        return tv.getDouble(0.0) == 1;
    }

    /**
     * Returns the area of the target, if detected.
     * @return 0.0 if no target, area otherwise
     */
    public double getArea() {
        return ta.getDouble(0.0);
    }

    /**
     * Gets the x-position of the target, if detected.
     * @return 0.0 if no target, x-position otherwise
     */
    public double getPosition() {
        return tx.getDouble(0.0);
    }

    /**
     * Gets the angle (with respect to the horizontal axis) of the target, if detected.
     * @return 0.0 if no target, x-angle otherwise
     */
    public double getAngle() {
        double angle = ts.getDouble(0.0);
        if (angle < -45) {
            angle += 90;
        }
        return angle;
    }

    public void trackTarget() {
        if (this.hasTarget()) {
            double area = this.getArea();
            double posError = this.getPosition(); // how far we are from the target
            // the target value we are going to
            double posValue = posError * kPos * Math.sqrt(Helper.boundValue((area * kArea), 0, 1));

            // powers to set drivetrain to
            double left = Helper.boundValue((1/Math.sqrt(area)) * kDist + posValue);
            double right = Helper.boundValue((1/Math.sqrt(area)) * kDist + posValue);
            
            
        }
    }

    /**
     * Prints the position, angle, and target status, for use on the SmartDashboard.
     * @return "Position,Angle,TargetFound"
     */
    @Override
    public String toString() {
        String toString = "" + this.getPosition() + "," + this.getAngle() + "," + this.hasTarget();
        return toString;
    }
}
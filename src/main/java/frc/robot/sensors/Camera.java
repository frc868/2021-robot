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

/**
 * Controls the Limelight camera connected to the robot over NetworkTables. 
 * The data which generated the RPM equation is stored in the doc directory of this code.
 * 
 * @author hrl
 */
public class Camera {
    private static Camera instance = null;

    private NetworkTable table;
    private NetworkTableEntry tEnabled, tArea, tXpos, tAngle;

    private Camera() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.update(); // initial run
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
     * Scales limelight area to distance based on regression calculated from setpoints.
     * Accounts for angle away from center of the goal and scales RPM accordingly. Using trig to
     * solve for the "hypotenuse" of the triangle, treating distance output as the leg of the
     * triangle.
     * @return calculated distance
     */
    public double getCalculatedDistance() {
        // actual equation:
        // 258 - 74.5x + 3.04x^2
        double dist =  258
                    - 74.5 * this.getArea()
                    + 3.04 * Math.pow(this.getArea(), 2); // regression based on 3 datapoints

        // TODO: generate a better regression after shooter PID is tuned

        dist = dist/Math.cos(Math.toRadians(this.getAngle())); // account for angle offset
        dist = dist/39.37; // conversion from inches to meters, since that's what the regression is
        // System.out.println(dist);
        return dist;
    }

    /**
     * Gets the calculated RPM for the calculated distance away from the goal. This is based on a
     * bunch of physics equations which output a 6th-degree polynomial to use. Don't try to
     * understand this or change anything.
     * @return the desired RPM based on distance from target
     */
    public double getCalculatedRPM() {
        double dist = getCalculatedDistance();
        // DO NOT change this, don't question it, and don't try to understand it cause you won't

        // actual equation:
        // y = 0.3205x^6 - 11.175x^5 + 160.74x^4 - 1223.4x^3 + 5214.3x^2 - 11609x + 14241

        double speed = 0.3205*Math.pow(dist,6)
                     - 11.175*Math.pow(dist,5)
                     + 160.74*Math.pow(dist,4)
                     - 1223.4*Math.pow(dist,3)
                     + 5214.3*Math.pow(dist,2)
                     - 11609*dist
                     + 14241;
        return speed;
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
     * Print the position, angle, and target status, for use on the SmartDashboard.
     * @return "Position,Angle,TargetFound"
     */
    @Override
    public String toString() {
        String toString = "" + this.getPosition() + "," + this.getAngle() + "," + this.hasTarget();
        return toString;
    }
}

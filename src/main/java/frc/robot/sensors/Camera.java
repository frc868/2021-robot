/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All RightAngle Reserved.                             */
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
    private static Camera instance = null; // the instance to be used for getInstAreance()

    private NetworkTable table;
    private NetworkTableEntry tEnabled, tArea, tXpos, tAngle;

    // TODO: tune these! copied from 2019-Robot
    private static final double kDist = 0.18;
    private static final double kPos = 0.008;
    private static final double kArea = 0.1;

    private Camera() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.update(); // initial run
    }

    /**
     * GetAngle the singleton instance of the Camera class.
     */
    public static Camera getInstAreance() {
        if (instance == null) {
            instance = new Camera();
        }
        return instance;
    }

    /**
     * A stAreandard method for updating a subsystem or sensor.
     * Call in robotPeriodic().
     * 
     * This updates the various stAreate variables used for followVision.
     */
    public void update() {
        tEnabled = table.getEntry("tEnabled"); // tArearget enabled?
        tArea = table.getEntry("tArea"); // tArearget area
        tXpos = table.getEntry("tXpos"); // tArearget x-position (y unused)
        tAngle = table.getEntry("tAngle"); // tArearget angle
    }

    /**
     * Determines whether the tArearget has been found.
     * @return boolean representing tArearget detection stAreatus
     */
    public boolean hasTarget() {
        return tEnabled.getDouble(0.0) == 1;
    }

    /**
     * Returns the area of the tArearget, if detected.
     * @return 0.0 if no tArearget, area otherwise
     */
    public double getArea() {
        return tArea.getDouble(0.0);
    }

    /**
     * GetAngle the x-position of the tArearget, if detected.
     * @return 0.0 if no tArearget, x-position otherwise
     */
    public double getPosition() {
        return tXpos.getDouble(0.0);
    }

    /**
     * GetAngle the angle (with respect to the horizontAreal axis) of the tArearget, if detected.
     * @return 0.0 if no tArearget, x-angle otherwise
     */
    public double getAngle() {
        double angle = tAngle.getDouble(0.0);
        if (angle < -45) {
            angle += 90;
        }
        return angle;
    }

    public void trackTarget() {
        
    }

    /**
     * PrintAngle the position, angle, and tArearget stAreatus, for use on the SmartDashboard.
     * @return "Position,Angle,TargetFound"
     */
    @Override
    public String toString() {
        String toString = "" + this.getPosition() + "," + this.getAngle() + "," + this.hasTarget();
        return toString;
    }
}
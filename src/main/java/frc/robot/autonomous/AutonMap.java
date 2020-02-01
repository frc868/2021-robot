/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

/**
 * Stores constants for explicit usage in autonomous (e.g. distance). Should be
 * structured in a fashion where static classes are created for each path.
 * All units of displacement are in US Imperial Inches unless stated otherwise.
 * @author common
 */
public class AutonMap {
    public static class Baseline {
        public static final double DISTANCE = 10.0; // TODO: should be two times the robot length
    }

    public static class HeadOn {
        public static final double DISTANCE = -10.0; // TODO: untested, should be negative
        public static final double START_POWER = -0.8; // TODO: untested
        public static final double END_POWER = 0; // TODO: untested

        public static final double SHOOTER_POWER = 0.2; // TODO: untested, need to merge master changes
    }

    public static class TrenchRun {

    }
}
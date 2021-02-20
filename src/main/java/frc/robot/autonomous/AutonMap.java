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
 * All units of power are from -1 to 1 unless stated otherwise.
 * All angles are in degrees unless stated otherwise.
 * @author common
 */
public class AutonMap {
    public static class Baseline {
        // the distance to drive from the baseline
        public static final double DISTANCE = 10.0;
        // the power to start driving with
        public static final double START_POWER = 0.5;
        // the power to end driving with
        public static final double END_POWER = 0;
    }

    public static class HeadOn {
        // the distance to drive (backwards) from the baseline
        // NOTE: since IPT is 1 (@Bloomington) this is actually encoder ticks
        public static final double DISTANCE = 10.0;
        // the power to start driving with
        public static final double START_POWER = 0.5;
        // the power to end driving with
        public static final double END_POWER = 0;

        // the backup (not PID) dead-reckoned power to run the shooter at
        public static final double BACKUP_SHOOTER_POWER = 0.73; // TODO: this should not be used
        // the power to run the shooter at
        public static final double SHOOTER_RPM = 4000; // TODO: tune
    }
    public static class Test{
        public static final double DISTANCE = 10000.0;
        public static final double START_POWER = 1;
        public static final double END_POWER = 0;
    }
    public static class AutonPathAB{
        public static final double START_POWER = 0.6;
        public static final double END_POWER = 0.0;
        public static class Angles{
            public static final double ANGLE_B3 = 90.0;
            public static final double ANGLE_C3 = -63.43;
            public static final double ANGLE_D5 = 18.43;
            public static final double ANGLE_E6 = -45.0;
            public static final double ANGLE_A6 = 45.0;
            public static final double ANGLE_B7 = -45.0;
            public static final double ANGLE_B8 = 45.0;
            public static final double ANGLE_D10 = -45.0;
        }
        public static class Distances{
            public static final double DISTANCE_B3 = 0.0; //untested
            public static final double DISTANCE_C3 = 0.0; //untested
            public static final double DISTANCE_D5 = 0.0; //untested
            public static final double DISTANCE_E6 = 0.0; //untested
            public static final double DISTANCE_A6 = 0.0; //untested
            public static final double DISTANCE_B7 = 0.0; //untested
            public static final double DISTANCE_B8 = 0.0; //untested
            public static final double DISTANCE_D10 = 0.0; //untested
            public static final double DISTANCE_D11 = 0.0; //untested
            
        }
    }
}

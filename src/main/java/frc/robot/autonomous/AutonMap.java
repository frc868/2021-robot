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
    // MEGA UBER TODO: tune ALL of these

    public static class Baseline {
        public static final double DISTANCE = 10.0; // the distance to drive from the baseline
        public static final double START_POWER = 0.8; // the power to start driving with
        public static final double END_POWER = 0; // the power to end driving with
    }

    public static class HeadOn {
        public static final double DISTANCE = -10.0; // the distance to drive (backwards) from the baseline
        public static final double START_POWER = -0.8; // the power to start straight driving with
        public static final double END_POWER = 0; // the power to end straight driving at

        public static final double SHOOTER_POWER = 0.2; // the power to run the shooter at
                                                        // TODO: this should probably be PID
    }

    public static class TrenchRun {
        public static final double START_POWER = 0.8; // the power to start straight driving with
        public static final double END_POWER = 0; // the power to end straight driving with
        public static final double INTAKE_POWER = 1; // the power to run the intake at
        public static final double SHOOTER_POWER = 0.2; // the power to run the shooter at
                                                        // TODO: again, should be PID

        public static final double DISTANCE_TO_BALLS = 10.0; // from starting position to the two balls
        public static final double DISTANCE_TO_TARGET = 10.0; // from ball position to the target

        public static final int TURN_ANGLE = 120; // the angle to turn from the ball intake position
    }
}
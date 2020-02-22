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
    // MEGA UBER TODO: tune ALL of these
    public static class Baseline {
        // the distance to drive from the baseline
        public static final double DISTANCE = 10.0;
        // the power to start driving with
        public static final double START_POWER = 0.8;
        // the power to end driving with
        public static final double END_POWER = 0;
    }

    public static class HeadOn {
        // the distance to drive (backwards) from the baseline
        public static final double DISTANCE = 10.0;
        // the power to start driving with
        public static final double START_POWER = -0.8;
        // the power to end driving with
        public static final double END_POWER = 0;

        // the power to run the shooter at
        // NOTE: this will be PID controlled and obsoleted in future code!
        public static final double SHOOTER_POWER = 0.8;
    }

    public static class TrenchRun {
        // from starting position to the balls we want to intake
        public static final double DISTANCE_TO_BALLS = 50.0;
        // from turned ball position to the target, regardless of angle
        public static final double DISTANCE_TO_TARGET = 50.0;
        // the angle to turn after
        public static final int TURN_ANGLE = 120;

        // the power to start driving with
        public static final double START_POWER = 0.8;
        // the power to end driving with
        public static final double END_POWER = 0;

        // the power to run the intake at
        public static final double INTAKE_POWER = 1;
        // the maximum amount of time (in seconds) we want to run the intake
        public static final double INTAKE_DELAY = 1.5;

        // the power to run the shooter at
        // NOTE: this will be PID controlled and obsoleted in future code!
        public static final double SHOOTER_POWER = 0.8;
    }

    public static class SixBall {
        // from starting position to the balls we want to intake
        public static final double DISTANCE_TO_BALLS = 50.0;
        // from ball position to the end of all three trench balls
        public static final double DISTANCE_WHILE_BALLS = 50.0;
        // from turned ball position to the target, regardless of angle
        public static final double DISTANCE_TO_TARGET = 50.0;
        // the angle to turn after
        public static final int TURN_ANGLE = 120;

        // the power to start driving with
        public static final double START_POWER = 0.8;
        // the power to end driving with
        public static final double END_POWER = 0;

        // the power to run the intake at
        public static final double INTAKE_POWER = 1;
        // the maximum amount of time (in seconds) we want to run the intake
        public static final double INTAKE_DELAY = 1.5;

        // the power to run the shooter at
        // NOTE: this will be PID controlled and obsoleted in future code!
        public static final double SHOOTER_POWER = 0.8;
    }
}

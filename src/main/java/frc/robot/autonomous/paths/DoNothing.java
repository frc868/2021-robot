/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.paths;

import frc.robot.Robot;

/**
 * Literally does nothing. I'm serious.
 * @author jk
 */
public class DoNothing {
    /**
     * Fails, successfully.
     */
    public void run() {
        Robot.drivetrain.setSpeed(0, 0);
    }

    /**
     * Pointless.
     */
    public void stop() {
        // useless here
    }
}

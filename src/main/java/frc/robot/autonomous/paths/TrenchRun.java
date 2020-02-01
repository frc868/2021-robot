/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.paths;

/**
 * Add your docs here.
 * 
 * 
 */

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class TrenchRun {
    private double distance;
     
    private TrenchRun(double distance) {
        this.distance = distance;
    }

    private final double startPwr = 1; //TODO: untested
    private final double endPwr = 2; //TODO: untested

    public void execute(){
        Robot.drivetrain.driveStraight(distance, startPwr, endPwr);
    }
}
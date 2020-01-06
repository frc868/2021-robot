/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.helpers;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * A wrapper to create a Command from nothing but a single function using lambdas.
 * Very useful for button mappings.
 * 
 * This will be deprecated when 2020 WPILib releases, as this functionality is built in.
 * 
 * @author hrl
 */
public class RunCommand extends InstantCommand {
    private Runnable body;

    public RunCommand(Runnable body) {
        this.body = body;
    }

    @Override
    protected void initialize() {
        body.run();
    }
}

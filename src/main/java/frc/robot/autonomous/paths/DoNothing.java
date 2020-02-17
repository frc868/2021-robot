package frc.robot.autonomous.paths                                  ;

import frc.robot.Robot                                              ;
import frc.robot.autonomous.AutonPath;

/**
 * <i>python</i>
 * @author you know who to blame for this one
 */
public class DoNothing extends AutonPath                            {
    private final double DONT_SPEED = 0                             ;
    @Override
    public void run()                                               { 
        dont()                                                      ;}
    private void dont()                                             { 
        Robot.drivetrain.setSpeed(DONT_SPEED, DONT_SPEED)           ; 
        System.out.println("!!!! DEPLOYING EVASIVE MANEUVERS !!!!") ;}}

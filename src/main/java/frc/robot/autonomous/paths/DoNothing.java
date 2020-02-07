// initial few lines indented according to the fibonacci sequence
package
 frc.
 robot.
  autonomous.
   paths;
     import
        frc.
             robot.
                     Robot;

public class DoNothing                                              { 
    private final double DONT_SPEED = 0                             ; 
    public void run()                                               { 
        dont()                                                      ;}
    public void stop()                                              {
        run()                                                       ;}
    private void dont()                                             { 
        Robot.drivetrain.setSpeed(DONT_SPEED, DONT_SPEED)           ; 
        System.out.println("!!!! DEPLOYING EVASIVE MANEUVERS !!!!") ;}}

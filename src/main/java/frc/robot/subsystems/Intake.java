package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The Hopper subsystem consists of 3 motors to move the power cells in stages. Positions one and
 * two are on the bottom, position three is the transition to the upper level of the hopper, and
 * positions four and five are on the top level.
 * 
 * @author dri, JW, AF
 */

public class Intake {
    private static Intake instance;

    private DigitalInput botLeft = new DigitalInput(0);
    private DigitalInput botRight = new DigitalInput(1);

    // private DigitalInput midLeft = new DigitalInput(2);
    // private DigitalInput midRight = new DigitalInput(3);

    private DigitalInput topLeft = new DigitalInput(4);
    private DigitalInput topRight = new DigitalInput(5);

    private XboxController controller = new XboxController(0);

    private WPI_TalonSRX intake = new WPI_TalonSRX(2);
    private WPI_TalonSRX middle = new WPI_TalonSRX(0);
    private WPI_TalonSRX outtake = new WPI_TalonSRX(21);

    private int count = 0;
    private int prev_count = 0;

    private Intake() {}

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public void initialize() {
        middle.setInverted(true);
        intake.setInverted(true);
        outtake.setInverted(true);
    }

    //in millaseconds
    int mid_cycle_time = 100000;
    int chain_cycle_time = 100000; 

    public void update() { //TODO: add logic back in
        
        if(prev_count < count) {  //add a ball
            intake.set(-1); 
            middle.set(-1);
            outtake.set(-1);
            Thread.sleep(mid_cycle_time/2);
            middle.set(0);
            Thread.sleep(chain_cycle_time-mid_cycle_time/2);
            outtake.set(0);
            intake.set(0);

            prev_count = count;
        } else if(prev_count > count) {  // want to remove a ball
            for(int i = 0;i<=5-count;i++)
            {
                intake.set(-1); 
                middle.set(-1);
                outtake.set(-1);
                Thread.sleep(mid_cycle_time/2);
                middle.set(0);
                Thread.sleep(chain_cycle_time-mid_cycle_time/2);
                outtake.set(0);
                intake.set(0);
            }
            //ping shooter ready
            //or
            //set shooter to delay -- chain_cycle_time * i calculation (extra forloop) 
            count --;
            prev_count = count;

        }
    }

    public boolean readyToShoot() {
        return top();
    }

    private boolean bottom() {
        count++;
        return botLeft.get() || botRight.get();
    }

    private boolean top() {
        return topLeft.get() || topRight.get();
    }

    public void driveStraight(double targetDist) {

    }
}
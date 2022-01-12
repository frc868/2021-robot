package frc.robot.helpers;
import edu.wpi.first.wpilibj.Joystick;
public class JoystickButtonWrapper {
    private final Joystick joystick;
    private final int id; //Button number, found expirimentally using the drivers station
    private boolean lastState = false;
    public JoystickButtonWrapper(Joystick stick, int id){
        this.joystick = stick;
        this.id = id;
    }
    public boolean get(){
        return this.joystick.getRawButton(this.id);
    }
    public void whenPressed(Runnable lamda) {
        if (this.get() && !lastState) {
            lamda.run();
            lastState = true;
        }
    }
    public void whenReleased(Runnable lamda) {
        if (!this.get() && lastState) {
            lamda.run();
            lastState = false;
        }
    }
    public void whileHeld(Runnable lamda) {
        if (this.get()) {
            lamda.run();
            lastState = true;
        }
    }
}

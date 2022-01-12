package frc.robot.helpers;

import edu.wpi.first.wpilibj.Joystick;
//Class used to make the joystick go burrrrrrrrrrr
public class JoystickWrapper {
    private final Joystick joystick;
    private final JoystickButtonWrapper bOne, bTwo, bThree, bFour, bFive, bSix, bSeven, bEight, bNine, bTen, bEleven, bTweleve;
    
    public JoystickWrapper(Joystick stick){
        this.joystick = stick;
        this.bOne = new JoystickButtonWrapper(this.joystick, 0);
        this.bTwo = new JoystickButtonWrapper(this.joystick, 1);
        this.bThree = new JoystickButtonWrapper(this.joystick, 2);
        this.bFour = new JoystickButtonWrapper(this.joystick, 3);
        this.bFive = new JoystickButtonWrapper(this.joystick, 4);
        this.bSix = new JoystickButtonWrapper(this.joystick, 5);
        this.bSeven = new JoystickButtonWrapper(this.joystick, 6);
        this.bEight = new JoystickButtonWrapper(this.joystick, 7);
        this.bNine = new JoystickButtonWrapper(this.joystick, 8);
        this.bTen = new JoystickButtonWrapper(this.joystick, 9);
        this.bEleven = new JoystickButtonWrapper(this.joystick, 10);
        this.bTweleve = new JoystickButtonWrapper(this.joystick, 11);
        
    }

    public double[] getVector(){
        double magnitude = this.joystick.getMagnitude();
        double angle = this.joystick.getDirectionRadians(); // must be in radians cause Java is stupid and Math.cos() and Math.sin() only work with radians
        //calculate the x and y components of the vector
        double x = magnitude * Math.cos(angle);
        double y = magnitude * Math.sin(angle);
        double[] vector = {x,y};
        return vector;
    }
}

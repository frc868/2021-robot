package frc.robot.autonomous.paths;

import frc.robot.autonomous.AutonPath;
import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;
public class AutonPathAB extends AutonPath {
    private PathABState curretnState = PathABState.Start;
    private static double currentDistance = 0;
    private static double currentAngle = 0;
    private static void resetCurrentDistance(){
        currentDistance = 0.0;
    }
    private enum PathABState{
        Start{
            @Override
            public PathABState nextState(){
                return FirstForward;
            }
            @Override
            public void run(){

            }
        },
        FirstForward{
            @Override
            public PathABState nextState(){
                return FirstTurn;
            }
            @Override
            public void run(){
                
            }

        },
        FirstTurn{ // 110 degrees
            @Override
            public PathABState nextState(){
                return SecondForward;
            }
            @Override
            public void run(){
                
            }

        },
        SecondForward{
            @Override
            public PathABState nextState(){
                return SecondTurn;
            }
            @Override
            public void run(){
                
            }

        },
        SecondTurn{
            @Override
            public PathABState nextState(){
                return Final;
            }
            @Override
            public void run(){
                
            }

        },
        Final{
            @Override
            public PathABState nextState(){
                return this;
            }
            @Override
            public void run(){
                
            }

        };



        public abstract PathABState nextState();
        public abstract void run();
       
        public void stop(){
            Robot.drivetrain.setSpeed(0,0);
        }
        

    }
    @Override
    public void run(){
        currentDistance = Robot.drivetrain.getCurrentDistance();
        currentAngle = Robot.gyro.getAngle();
        this.curretnState.run();
        this.curretnState.nextState();
    }
    @Override
    public void reset(){

    }
    @Override
    public String toString(){
        return "Authon Path AB";
    }
}

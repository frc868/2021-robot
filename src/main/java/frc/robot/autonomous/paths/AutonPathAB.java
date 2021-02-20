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
                return ToB3;
            }
            public void run(){
                Robot.drivetrain.setSpeed(0, 0);
                System.out.println("Start");
            }
        },
        ToB3{
            @Override
            public PathABState nextState(){
                if(currentDistance < AutonMap.AutonPathAB.Distances.DISTANCE_B3){
                    return this;
                }
                return TurnB3;
            }
            @Override
            public void run(){
                resetCurrentDistance();
                Robot.drivetrain.driveStraight(AutonMap.AutonPathAB.Distances.DISTANCE_B3, AutonMap.AutonPathAB.START_POWER, AutonMap.AutonPathAB.END_POWER);
                System.out.println("ToB3");
            }
        },
        TurnB3{
            @Override
            public PathABState nextState(){
                if(currentAngle < AutonMap.AutonPathAB.Angles.ANGLE_B3){
                    return this;
                }
                return ToC3;
            }
            @Override
            public void run(){
                Robot.gyro.reset();
                Robot.turn.run(AutonMap.AutonPathAB.Angles.ANGLE_B3);
                System.out.println("TurnB3");

            }

        },
        ToC3{
            @Override
            public PathABState nextState(){
                return TurnC3;
            }
            @Override
            public void run(){

            }

        },
        TurnC3{
            @Override 
            public PathABState nextState(){
                return ToD5;
            }
            @Override
            public void run(){

            }

        },
        ToD5{
            @Override
            public PathABState nextState(){
                return TurnD5;
            }
            @Override
            public void run(){

            }

        },
        TurnD5{
            @Override
            public PathABState nextState(){
                return ToE6;
            }
            @Override
            public void run(){

            }

        },
        ToE6{
            @Override
            public PathABState nextState(){
                return TurnE6;
            }
            @Override
            public void run(){

            }

        },
        TurnE6{
            @Override
            public PathABState nextState(){
                return ToA6;
            }
            @Override
            public void run(){

            }

        },
        ToA6{
            @Override
            public PathABState nextState(){
                return TurnA6;
            }
            @Override
            public void run(){

            }

        },
        TurnA6{
            @Override
            public PathABState nextState(){
                return ToB7;
            }
            @Override
            public void run(){

            }

        },
        ToB7{
            @Override
            public PathABState nextState(){
                return TurnB7;
            }
            @Override
            public void run(){

            }

        },
        TurnB7{
            @Override
            public PathABState nextState(){
                return ToB8;
            }
            @Override
            public void run(){

            }

        },
        ToB8{
            @Override
            public PathABState nextState(){
                return TurnB8;
            }
            @Override
            public void run(){

            }

        },
        TurnB8{
            @Override
            public PathABState nextState(){
                return ToD10;
            }
            @Override
            public void run(){

            }

        },
        ToD10{
            @Override
            public PathABState nextState(){
                return TurnD10;
            }
            @Override
            public void run(){

            }

        },
        TurnD10{
            @Override
            public PathABState nextState(){
                return ToD11;
            }
            @Override
            public void run(){

            }

        },
        ToD11{
            @Override
            public PathABState nextState(){
                return Finish;
            }
            @Override
            public void run(){

            }
        },
        Finish{
            @Override
            public PathABState nextState(){
                return this;
            }
            @Override
            public void run(){
                System.out.println("Finished");

            }

        };


        public abstract PathABState nextState();
        public abstract void run();
       
        public void stop(){

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

package frc.robot.autonomous.paths;
import frc.robot.Robot;
import frc.robot.autonomous.AutonMap;
import frc.robot.autonomous.AutonPath;

public class Test extends AutonPath{
    private static double total_distance = 0;
    private TestState currentState = TestState.ToPosition;

    private enum TestState {
        ToPosition {
            @Override
            public TestState nextState() {
                if(total_distance < AutonMap.Test.DISTANCE){
                    return this;
                }
                return LeftTurn;
            }

            @Override
            public void run() {
                Robot.drivetrain.setSpeed(.5, .5);
            }

            @Override
            public String toString() {
                return "ToPosition";
            }
        },
        LeftTurn {
            @Override
            public TestState nextState() {
                 if(Robot.gyro.getAngle() < 90) {
                    return this;
                 }
                 return End;
            }

            @Override
            public void run(){
                Robot.drivetrain.turnLeft();
            }

            @Override
            public String toString() {
                return "LeftTurn";
            }
        },
        End {
            @Override
            public TestState nextState() {
                return this;
            }

            @Override
            public void run() {
                Robot.drivetrain.setSpeed(0, 0);
            }

            @Override
            public String toString() {
                return "Done";
            }
        };

        public abstract TestState nextState();
        public abstract void run();
    }

    @Override
    public void run(){
        total_distance = Robot.drivetrain.getCurrentDistance();
        this.currentState.run();
        this.currentState = this.currentState.nextState();
    }

    @Override
    public String toString(){
        return "Test";
    }

    @Override
    public void reset(){
        this.currentState = TestState.ToPosition;
    }
    
}

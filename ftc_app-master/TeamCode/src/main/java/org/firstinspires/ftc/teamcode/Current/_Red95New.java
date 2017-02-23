package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "R 95", group = "New")
public class _Red95New extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(_Red95New.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.ShootByVoltage();
        robot.Move(55, 1.0);
        sleep(500);
        robot.EnableShot();
        robot.StopShooter();
        robot.DiagonalForwardsLeftCoast(50, 1);
        robot.DiagonalForwardsLeft(20, .75, 1);
        robot.AlignToWithin(1.5, .05);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(2, .5, .05);
        if(!robot.FindAndPressForwards(Robot.team.Red)){
            robot.Move(20, -.5);
            robot.AlignToWithin(1, .05);
            robot.StrafeToWall(10, .10);
            robot.ForwardsPLoop(10, .25);
            robot.FindAndPressBackwards(Robot.team.Red);
        }
        robot.CheckBeacon(Robot.team.Red);
        robot.Move(85, -1.0);
        robot.StrafeFromWall(10, .5);
        robot.AlignToWithin(3, .05);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(-2, .5, .05);
        if(!robot.FindAndPressBackwards(Robot.team.Red)){
            robot.Move(20, .5);
            robot.AlignToWithin(1, .05);
            robot.StrafeToWall(10, .10);
            robot.BackwardsPLoop(30, .25);
            robot.FindAndPressForwards(Robot.team.Red);
        }
        robot.CheckBeacon(Robot.team.Red);

        robot.ArcadeToAngleLeft(0, .25, -.40, 7.5);
        robot.AlignToWithinOf(-17.5, 1, .05);
        robot.MoveCoast(100, - .5);
        robot.Move(25, - .25);
//        robot.MoveToPitch(10, -.25);
    }
}

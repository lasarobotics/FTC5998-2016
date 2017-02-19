package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "B 100 New", group = "New")
public class _Blue100New extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeNoBottom(_Blue100New.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.MoveCoast(7.5, -1.0);
        robot.DiagonalBackwardsLeftCoast(80, 1);
        robot.DiagonalBackwardsLeft(25, .75, 1);
        robot.AlignToWithin(3, .05);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithin(1, .05);
        robot.StrafeToWall(9, .10);
        robot.AlignToWithin(.75, .05);
        robot.StrafeToWall(9, .10);
        robot.AlignToWithin(.5, .05);
        robot.StrafeToWall(8, .10);
        robot.FindAndPress(Robot.team.Blue, .15);
        robot.CheckBeacon(Robot.team.Blue);

        robot.AlignToWithin(.5, .05);
        robot.BackwardsPLoop(100, -1);
        robot.AlignToWithin(1, .05);
        robot.StrafeToWall(9, .10);

        robot.FindAndPressThenCorrect(Robot.team.Blue, - .15);
        robot.CheckBeacon(Robot.team.Blue);
        robot.ShootByVoltage();
        robot.ArcadeToAngleRight(0, .15, .30, 25);
        robot.AlignToWithinOf(40, 1, .05);
        robot.Move(130, 1.0);
        robot.AlignToWithinOf(40, 1, .05);
        //AND HERE
        robot.EnableShot();
        robot.StopShooter();
        robot.Move(80, 1.0);


    }
}

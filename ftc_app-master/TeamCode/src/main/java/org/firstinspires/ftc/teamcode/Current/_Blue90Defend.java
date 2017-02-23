package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "B 90 Defend", group = "New")
public class _Blue90Defend extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(_Blue90Defend.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.ShootByVoltage();
        robot.Move(55, 1.0);
        sleep(1000);
        robot.EnableShot();
        robot.StopShooter();
        if(robot.navX.getYaw() > 0){
            robot.TurnLeft(0, .2);
        } else {
            robot.TurnRight(0, .2);
        }
        robot.AlignToWithin(1, .10);
        robot.MoveCoast(20, -1.0);
        robot.DiagonalBackwardsLeftCoast(80, 1);
        robot.DiagonalBackwardsLeft(25, .75, 1);
        robot.AlignToWithin(1.5, .05);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(2, .5, .05);

        if(!robot.FindAndPressForwards(Robot.team.Blue)){
            robot.Move(20, - .5);
            robot.AlignToWithin(1, .05);
            robot.StrafeToWall(10, .10);
            robot.ForwardsPLoop(30, .25);
            robot.FindAndPressBackwards(Robot.team.Blue);
        }
        robot.CheckBeacon(Robot.team.Blue);
        robot.Move(85, -1.0);
        robot.StrafeFromWall(10, .5);
        robot.AlignToWithin(3, .05);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(-2, .5, .05);
        if(!robot.FindAndPressBackwards(Robot.team.Blue)){
            robot.Move(20, .5);
            robot.AlignToWithin(1, .05);
            robot.StrafeToWall(10, .10);
            robot.BackwardsPLoop(30, .25);
            robot.FindAndPressForwards(Robot.team.Blue);
        }
        robot.CheckBeacon(Robot.team.Blue);
        robot.ArcadeToAngleLeft(0, .25, .40, -15);
        robot.TurnLeft(100, .25);
        robot.AlignToWithinOf(135, 1, .05);
        robot.Move(20, -.5);
        robot.WaitForRange(10);
        robot.AlignToWithinOf(-25, 5, .15);
        robot.Move(30, .5);
        robot.Move(10, -.5);
        robot.Move(30, .5);


    }
}

package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "B 100", group = "New")
public class _Blue100New extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(_Blue100New.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.Housekeeping();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.MoveCoast(20, -1.0);
        robot.DiagonalBackwardsLeftCoast(80, 1);
        robot.DiagonalBackwardsLeft(25, .75, 1);
        robot.AlignToWithin(1.5, .05);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(1, .5, .05);

        if(!robot.FindAndPress(Robot.team.Blue, .15)){
            robot.FindAndPress(Robot.team.Blue, -.15);
        }
        robot.CheckBeacon(Robot.team.Blue);
        robot.Move(85, -1.0);
        robot.AlignToWithin(1, .05);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithinOf(-.25, 1, .05);
        if(!robot.FindAndPress(Robot.team.Blue, -.15)){
            robot.FindAndPress(Robot.team.Blue, .15);
        }
        robot.CheckBeacon(Robot.team.Blue);

        robot.ShootByVoltage();
        robot.ArcadeToAngleRight(0, .25, .40, 20);
        robot.AlignToWithinOf(40, 1, .05);
        robot.Move(135, 1.0);
        robot.AlignToWithinOf(40, 1, .05);
//        robot.BackwardsPLoop(5, 0.5);
        robot.EnableShot();robot.StopShooter();
        robot.Move(80, 1.0);


    }
}

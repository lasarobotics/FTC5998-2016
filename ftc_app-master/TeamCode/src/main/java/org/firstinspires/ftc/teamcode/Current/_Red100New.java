package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "R 100 New", group = "New")
public class _Red100New extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeNoBottom(_Red100New.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart();
        robot.ShootByVoltage();
        robot.Move(65, 1.0);
        robot.EnableShot();
        robot.StopShooter();
        robot.Move(10, - 0.75);
        boolean didCrash = robot.DiagonalForwardsLeft(45, 1);
        robot.StrafeToWall(25, .5);
        robot.AlignToWithin(1.5, .05);
        robot.StrafeToWall(10, .10);
        robot.AlignToWithin(.5, .05);

        if(!didCrash) {
            robot.FindAndPress(Robot.team.Red, .25);
            robot.BackwardsPLoop(70, .5);
            robot.FindAndPress(Robot.team.Red, - .10);
        } else {
            robot.FindAndPress(Robot.team.Red, - .10);
            robot.ForwardsPLoop(70, .5);
            robot.FindAndPress(Robot.team.Red, .10);
        }
        robot.StrafeFromWall(25, 1.0);
        if(!didCrash){
            robot.AlignToWithin(1.5, .05);
            robot.Move(115, -1.0);
            robot.Move(25, -0.25);
        } else {
            robot.TurnRightEnc(35, 0.50);
            robot.Move(15, 1.0);
            robot.Move(15, -.75);
            robot.Move(15, 0.50);
            robot.Move(15, -0.25);
        }

    }
}

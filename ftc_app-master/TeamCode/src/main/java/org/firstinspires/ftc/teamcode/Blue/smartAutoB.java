package org.firstinspires.ftc.teamcode.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "Smart B", group = "Blue")
public class smartAutoB extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(smartAutoB.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.Move(40, -1.0);
        robot.DiagonalBackwardsRight(10, 1.0);
        robot.AlignToWithin(2, .05);

        robot.StrafeToWall(10, .15);

        robot.AlignToWithin(2, .05);

        robot.LineSearch(2, .10);
        robot.PressBeacon(Robot.team.Blue);
        robot.StrafeFromWall(15, 1.0);
        robot.AlignToWithin(2, .05);

        robot.BackwardsPLoop(130, .75);
        robot.StrafeToWall(10, .15);
        robot.AlignToWithin(2, .05);

        robot.LineSearch(2, -.10);
        robot.PressBeacon(Robot.team.Blue);
        robot.StrafeFromWall(20, 1.0);
        robot.TurnRightRelative(45, .05);
        robot.ShootSmart();
        robot.Move(115, 1.0);
        robot.EnableShot(850, 1);
        robot.StopShooter();
        robot.Move(65, 1.0);


    }
}

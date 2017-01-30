package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "B Diagonal", group = "aaMain")
public class Blue100Points extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(Blue100Points.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.MoveCoast(15, -1.0);
        robot.DiagonalBackwardsLeftCoast(80, 1);
        robot.DiagonalBackwardsLeft(25, .75, 1);
        robot.AlignToWithin(3, .05);
        robot.StrafeToWall(12, .10);
        robot.AlignToWithin(1, .05);
        robot.LineSearch(2, .20);
        sleep(250);
        robot.LineSearch(2, - .05);
        robot.StrafeToWall(9, .10);
        robot.PressBeacon(Robot.team.Blue);

        robot.StrafeFromWall(11, .50);

        robot.AlignToWithin(1, .05);
        robot.Move(130, -1.0);
        robot.AlignToWithin(1.0, .05);

        robot.LineSearch(2, - .15);
        sleep(250);
        robot.LineSearch(2, .05);
        robot.StrafeToWall(9, .10);
        robot.PressBeacon(Robot.team.Blue);

        robot.StrafeFromWall(25, .75);
        robot.ShootSmart();
        robot.TurnRight(20, .50);
        robot.AlignToWithinOf(35, 1, .05);
        robot.Move(110, 1.0);
        robot.AlignToWithinOf(35, 1, .05);
        robot.EnableShot(250, 1.0);
        sleep(250);
        robot.EnableShot(750, 1.0);
        robot.StopShooter();
        robot.Move(75, 0.25);


    }
}

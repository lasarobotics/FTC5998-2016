package org.firstinspires.ftc.teamcode.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "B Diagonal", group = "Blue")
public class smartAutoB extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(smartAutoB.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.Move(60, -1.0);

        sleep(100);

        robot.DiagonalBackwardsLeft(80, 1);
        robot.DiagonalBackwardsLeft(25, .75, 1.0);
        robot.AlignToWithin(2, .05);
        robot.StrafeToPrecise(13, 1, .10);
        robot.AlignToWithin(2, .05);
//        robot.ForwardsPLoop(50, .75);

        robot.LineSearch(2, .10);
        robot.StrafeToWall(9, .10);
        robot.PressBeaconSimple(Robot.team.Blue);
        robot.StrafeToPrecise(13, 1, .10);

        robot.AlignToWithin(1, .05);
        robot.Move(130, -1.0);
        robot.LineSearch(2, -.10);
        robot.AlignToWithin(1, .05);

        robot.StrafeToWall(9, .10);
        robot.PressBeaconSimple(Robot.team.Blue);
        robot.StrafeFromWall(25, .75);
        robot.ShootSmart();
        robot.TurnRight(35, .25);
        robot.AlignToWithinOf(40, 1, .05);
        robot.Move(110, 1.0);
        robot.EnableShot(1000, 1.0);
        robot.StopShooter();
        robot.Move(65, 0.25);


    }
}

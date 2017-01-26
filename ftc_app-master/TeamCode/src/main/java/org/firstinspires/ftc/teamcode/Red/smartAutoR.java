package org.firstinspires.ftc.teamcode.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "Smart R", group = "Red")
public class smartAutoR extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(smartAutoR.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart();
        robot.ShootSmart();
        robot.Move(65, 1.0);
        robot.EnableShot(850, 1);
        robot.StopShooter();
        robot.DiagonalForwardsLeft(15, 1);

//        robot.StrafeFromWall(13, 1.0);
        robot.StrafeToPrecise(13, 1, .10);

        robot.AlignToWithin(2, .05);
        robot.LineSearch(2, .10);
        robot.StrafeToPrecise(8, 1, .10);
//        robot.StrafeToWall(9, .10);
        robot.PressBeaconSimple(Robot.team.Red);

        robot.StrafeToPrecise(13, 1, .10);
        robot.AlignToWithin(1, .05);
        robot.Move(130, -1.0);
        robot.LineSearch(2, -.10);
        robot.AlignToWithin(1, .05);

        robot.DiagonalForwardsLeft(9, .10);
        robot.LineSearch(2, -.10);

        robot.PressBeaconSimple(Robot.team.Red);
        robot.StrafeFromWall(25, .75);
        robot.Move(110, -1.0);
        robot.Move(25, -0.25);

    }
}

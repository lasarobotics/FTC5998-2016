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
        while(!isStarted()){
            robot.sensorsInfo(telemetry);
        }
        waitForStart();
        robot.ShootSmart();
        robot.ForwardsPLoop(40);
        robot.EnableShot(850, 1);
        robot.StopShooter();
        robot.DiagonalForwardsRight(150, 1);

        robot.AlignToWithin(2, .05);

        robot.StrafeToWall(10, .15);

        robot.AlignToWithin(2, .05);

        robot.LineSearch(2, -.10);
        robot.PressBeacon(Robot.team.Red);
        robot.StrafeFromWall(15, 1.0);
        robot.ForwardsPLoop(130, .75);
        robot.StrafeToWall(10, .15);
        robot.LineSearch(2, -.10);
        robot.PressBeacon(Robot.team.Red);
        robot.StrafeFromWall(20, 1.0);

    }
}

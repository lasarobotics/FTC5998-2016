package org.firstinspires.ftc.teamcode.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Blue", name = "B_S1B")
public class Blue1BeaconShoot extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(Blue1BeaconShoot.this, hardwareMap, telemetry, true);
        waitForStart();
        robot.Move(80, - 1.00);
        robot.TurnRight(30, 0.10);
        robot.Move(225,- 1.00);
        robot.AlignToWithin(3, 0.05);
        robot.StrafeToWall(23, 0.15);
        robot.AlignToWithin(3, 0.05);
        robot.LineSearch(2, 0.10);
        robot.StrafeToWall(10, 0.10);
        robot.LineSearch(2, - 0.10);
        robot.LineSearch(2, 0.05);
        robot.StrafeToWall(8, 0.10);
        robot.PressBeacon(Robot.team.Blue );

        robot.StrafeFromWall(20, 0.45);
        robot.ShootAtPower(0, 0.65);
        robot.TurnLeft(90, .10);
        robot.Move(15, 1.00);
        robot.EnableShot(1500, 0);
        robot.ShootAtPower(0, 0.0);
        robot.Move(45, 1.00);
    }
}

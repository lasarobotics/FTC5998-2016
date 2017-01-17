package org.firstinspires.ftc.teamcode.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Blue", name = "B_1B Extra")
public class Blue1B3Shoot extends LinearOpMode{
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(Blue1B3Shoot.this, hardwareMap, telemetry, true);
        waitForStart();

        robot.infeed.setPower(1);
        sleep(500);
        robot.infeed.setPower(0);
        robot.TurnRight(90, .35);
        robot.AlignToWithinOf(2.5, - 90, .15);
        robot.infeed.setPower(0);
        robot.Move(70, - 1.00);
        robot.TurnRight(55, 0.15);
        robot.Move(240, - 1.00);
        robot.AlignToWithinOf(3, - 90, .05);
        //Line up with the wall
        robot.StrafeToWall(15, 0.10);

        robot.AlignToWithinOf(2.5, - 90, .05);
        robot.AlignToWithinOf(2.5, - 90, .05);
        robot.LineSearch(2, - 0.10);
        robot.LineSearch(2,   0.05);
        robot.StrafeToWall(9, 0.10);

        robot.LineSearch(2,   0.10);
        robot.LineSearch(2, - 0.05);
        robot.LineSearch(2,   0.05);
        robot.PressBeacon(Robot.team.Blue );
        //Press the first beacon
        robot.StrafeFromWall(13, .20);
        robot.TurnRightRelative(30, .05);
        robot.ShootSmart();
        robot.Move(70, -1.0);
        robot.TurnRightRelative(45, .05);
        robot.EnableShot(1500, 1.0);
        robot.StopShooter();
        robot.Move(70, -1.0);

    }
}

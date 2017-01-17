package org.firstinspires.ftc.teamcode.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Red", name = "R_2B Shoot 3")
public class Red2B3Shoot extends LinearOpMode{
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(Red2B3Shoot.this, hardwareMap, telemetry, true);
        waitForStart();

        robot.infeed.setPower(1);
        sleep(500);
        robot.TurnLeft(90, .35);
        robot.infeed.setPower(0);
        robot.AlignToWithinOf(2.5, - 90, .15);
        robot.infeed.setPower(0);
        robot.ShootSmart();
//        robot.ShootAtPower(1, .65);
        robot.Move(70, 1.00);
        robot.EnableShot(1100, 1.00);
        robot.infeed.setPower(0);
        robot.shoot1.setPower(0);
        robot.shoot2.setPower(0);
        robot.TurnLeft(125, 0.15);
        robot.Move(240, 1.00);
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
        robot.PressBeacon(Robot.team.Red );
        //Press the first beacon
        robot.StrafeFromWall(15, 1.0);
        robot.AlignToWithinOf(2.5, - 90, .05);
        robot.AlignToWithinOf(2.5, - 90, .05);
        robot.Move(140, 1.00);
        robot.AlignToWithinOf(2.5, - 90, .05);
        robot.LineSearch(2, 0.11);
        robot.AlignToWithinOf(2.5, - 90, .05);
        robot.StrafeToWall(9, 0.10);
        robot.AlignToWithinOf(2.5, - 90, .05);
        robot.LineSearch(2,   0.10);
        robot.LineSearch(2, - 0.05);
        robot.PressBeacon(Robot.team.Red);
        //Press the second beacon

        robot.StrafeFromWall(13, 1.00);
        robot.TurnLeftEnc(35, 1.00);
        robot.Move(215, -1.00);
    }
}

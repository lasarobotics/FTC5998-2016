package org.firstinspires.ftc.teamcode.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Red", name = "R_2B (No Shoot)")
public class Red2BNoShoot extends LinearOpMode{
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(Red2BNoShoot.this, hardwareMap, telemetry, true);
        waitForStart();
        robot.Move(80, 1.00);
        robot.TurnLeft(35, 0.15);
        robot.Move(240, 1.00);
        robot.AlignToWithin(3, 0.05);
        //Line up with the wall
        robot.StrafeToWall(15, 0.10);

        robot.AlignToWithin(2.5, 0.05);
        robot.AlignToWithin(2.5, 0.05);
        robot.LineSearch(2, - 0.10);
        robot.LineSearch(2,   0.05);
        robot.StrafeToWall(9, 0.10);

        robot.LineSearch(2,   0.10);
        robot.LineSearch(2, - 0.05);

        robot.PressBeacon(Robot.team.Red );
        //Press the first beacon
        robot.StrafeFromWall(15, 1.0);
        robot.AlignToWithin(2, 0.05);
        robot.Move(140, 1.00);
        robot.AlignToWithin(2.5, 0.05);
        robot.LineSearch(2, 0.11);
        robot.AlignToWithin(2.5, 0.05);
        robot.StrafeToWall(9, 0.10);
        robot.AlignToWithin(2.5, 0.05);
        robot.LineSearch(2,   0.10);
        robot.LineSearch(2, - 0.05);
        robot.PressBeacon(Robot.team.Red);
        //Press the second beacon

        robot.StrafeFromWall(13, 1.00);
        robot.TurnLeftEnc(35, 1.00);
        robot.Move(215, -1.00);
    }
}

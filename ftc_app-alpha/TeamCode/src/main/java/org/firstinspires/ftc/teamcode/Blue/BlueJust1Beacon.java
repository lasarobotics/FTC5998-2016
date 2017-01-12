package org.firstinspires.ftc.teamcode.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Blue", name = "B_NS1B")
public class BlueJust1Beacon extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(BlueJust1Beacon.this, hardwareMap, telemetry, true);
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
    }
}

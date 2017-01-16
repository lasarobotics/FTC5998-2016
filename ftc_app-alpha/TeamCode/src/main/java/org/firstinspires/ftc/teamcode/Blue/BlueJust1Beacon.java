package org.firstinspires.ftc.teamcode.Blue;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Blue", name = "B_Just 1 Beacon")
public class BlueJust1Beacon extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(BlueJust1Beacon.this, hardwareMap, telemetry, true);
        waitForStart();
        robot.Move(40, - 1.00);
        robot.TurnRight(35, 0.15);
        robot.Move(240, - 1.00);
        robot.AlignToWithin(3, 0.05);
        //Line up with the wall
        robot.StrafeToWall(15, 0.10);

        robot.AlignToWithin(2.5, 0.05);
        robot.AlignToWithin(2.5, 0.05);
        robot.LineSearch(2, 0.10);
        robot.LineSearch(2, - 0.05);
        robot.AlignToWithin(2.5, 0.05);
        robot.AlignToWithin(2.5, 0.05);
        robot.StrafeToWall(9, 0.10);

        robot.PressBeacon(Robot.team.Blue );
        //Press the first beacon

    }
}

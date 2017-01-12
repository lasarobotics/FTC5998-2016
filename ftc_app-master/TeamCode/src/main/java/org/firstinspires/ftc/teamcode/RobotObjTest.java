package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(name="Object Test", group="Autonomous")
public class RobotObjTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();
        robot.initialize(RobotObjTest.this, hardwareMap, telemetry, true);
        waitForStart();
        telemetry.addData("S", 2);
        telemetry.update();
        robot.TurnLeft(45, .15);

        telemetry.addData("S", 3);
        telemetry.update();
        robot.TurnRight(45, .15);

        telemetry.addData("S", 4);
        telemetry.update();
        robot.TurnLeftAbsolute(-90, .15);

        telemetry.addData("S", 5);
        telemetry.update();

        robot.TurnRightRelative(45, .15);

        telemetry.addData("S", 6);
        telemetry.update();
        robot.TurnLeftRelative(45, .15);
    }
}

package org.firstinspires.ftc.teamcode.Neutral;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */
@Autonomous(group = "ZNeutral", name = "CloseNoPark")
public class ShootCloseNoPark extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(ShootCloseNoPark.this, hardwareMap, telemetry, false);
        waitForStart();
        sleep(1000*10);
        robot.Move(105, .5);
        robot.ShootSmart();
        robot.EnableShot(2000, 1.0);
        robot.StopShooter();
        robot.Move(60, .5);
        sleep(1000);
        robot.Move(30, -.5);
    }
}

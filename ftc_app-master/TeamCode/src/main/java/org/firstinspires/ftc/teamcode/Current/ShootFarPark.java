package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "ZNeutral", name = "FarPark")
public class ShootFarPark extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(ShootFarPark.this, hardwareMap, telemetry, false);
        waitForStart();
        sleep(1000*10);
        robot.Move(180, .5);
        robot.ShootSmart();
        robot.EnableShot(2000, 1.00);
        robot.StopShooter();
        robot.Move(60, .5);
        sleep(1000);
        robot.Move(30, -.5);
        robot.Move(30, .5);
    }
}

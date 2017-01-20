package org.firstinspires.ftc.teamcode.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "YNeutral", name = "R_FarParkRamp")
public class ShootFarParkRampR extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(ShootFarParkRampR.this, hardwareMap, telemetry, true);
        waitForStart();
        sleep(1000*5);
        robot.Move(180, .5);
        robot.ShootSmart();
        robot.EnableShot(2000, 1.00);
        robot.StopShooter();
        robot.TurnLeft(45, .05);
        robot.Move(90, .5);
        robot.TurnLeftEnc(20, .50);
        robot.Move(50, .5);
    }
}

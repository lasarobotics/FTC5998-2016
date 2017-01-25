package org.firstinspires.ftc.teamcode.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "YNeutral", name = "B_FarParkRamp")
public class ShootFarParkRampB extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(ShootFarParkRampB.this, hardwareMap, telemetry, true);
        waitForStart();
        sleep(1000*5);
        robot.Move(180, .5);
        robot.ShootSmart();
        robot.EnableShot(2000, 1.00);
        robot.StopShooter();
        robot.TurnRight(45, .05);
        robot.Move(90, .5);
        robot.TurnRightEnc(20, .50);
        robot.Move(110, .5);
    }
}

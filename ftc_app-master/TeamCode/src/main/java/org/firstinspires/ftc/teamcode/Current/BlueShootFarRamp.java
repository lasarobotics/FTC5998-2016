package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "Neutral", name = "B_FarParkRamp")
public class BlueShootFarRamp extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeNoBottom(BlueShootFarRamp.this, hardwareMap, telemetry, true);
        waitForStart();
        sleep(1000*5);
        robot.ShootByVoltage();
        robot.Move(180, .5);
        robot.EnableShot(250, 1);
        robot.StopShooter();
        robot.TurnRightEnc(45, .05);
        robot.Move(110, .5);
        robot.TurnRightEnc(20, .50);
        robot.Move(110, .5);
        robot.Move(20, .25);
    }
}

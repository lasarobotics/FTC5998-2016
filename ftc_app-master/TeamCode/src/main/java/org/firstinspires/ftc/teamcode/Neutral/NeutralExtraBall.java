package org.firstinspires.ftc.teamcode.Neutral;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Neutral", name = "3 ball Extra")
public class NeutralExtraBall extends LinearOpMode{
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(NeutralExtraBall.this, hardwareMap, telemetry, true);
        waitForStart();

        robot.infeed.setPower(1);
        sleep(1000);
        robot.TurnLeft(90, .35);
        robot.infeed.setPower(0);
        robot.AlignToWithinOf(2.5, - 90, .15);
        robot.infeed.setPower(0);
        robot.ShootSmart();
        robot.Move(70, 1.00);
        robot.EnableShot(1100, 1.00);
        robot.infeed.setPower(0);
        robot.shoot1.setPower(0);
        robot.shoot2.setPower(0);
        robot.TurnLeft(100, 0.15);
        robot.Move(70, 1.00);
        sleep(1000);
        robot.Move(30, - 1.00);
        robot.Move(40, 1.00);

    }
}

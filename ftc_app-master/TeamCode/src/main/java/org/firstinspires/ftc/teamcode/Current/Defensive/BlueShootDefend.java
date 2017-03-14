package org.firstinspires.ftc.teamcode.Current.Defensive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Robot;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "Neutral", name = "B_Defend")
public class BlueShootDefend extends LinearOpMode {
    Robot robot = new Robot();
    boolean Forwards = true;
    boolean lastA = false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(BlueShootDefend.this, hardwareMap, telemetry, true);
        while (!isStarted() && !isStopRequested()) {
            if(gamepad1.a && !lastA){
                Forwards = !Forwards;
            }
            lastA = gamepad1.a;
            telemetry.addData("Route", Forwards ? "Forwards to Wall" : "Strafe to Wall");
            robot.sensorsInfo();
        }
        waitForStart();
        double stTime = getRuntime();
        robot.ShootByVoltage();
        robot.ForwardsPLoop(180, 1);

        robot.EnableShot(250, 1);
        robot.StopShooter();

        robot.Move(115, 1);
        robot.AlignToWithin(5, .15);
        robot.StrafeRight(10, .5);
        robot.AlignToWithin(1.5, .05);
        while ((getRuntime() - stTime) < 10) {
            sleep(1);
        }
        robot.Move(215, 1);
        sleep(4500);
        robot.Move(2.54*36, -1);
        sleep(1000);
        robot.Move(2.54*36, 1);
        sleep(4500);
        robot.Move(2.54*36, -1);
        sleep(1000);
        robot.Move(180, 1);

    }
}

package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ethan Schaffer on 1/12/2017.
 */

@Autonomous(group = "Neutral", name = "B_FarDefend")
public class BlueShootDefend extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(BlueShootDefend.this, hardwareMap, telemetry, true);
        while(!isStarted() && !isStopRequested()){
            robot.sensorsInfo();
        }
        waitForStart();
        double stTime = getRuntime();
        robot.ShootByVoltage();
        robot.Move(180, .5);
        robot.EnableShot(250, 1);
        robot.StopShooter();
        robot.Move(20, -.50);
        robot.AlignToWithinOf(-45, 3, .05);
        while((getRuntime() - stTime) < 10){
            sleep(1);
        }
        robot.Move(110, .5);
        robot.TurnRight(70, .15);
        robot.AlignToWithinOf(90, 3, .05);
        robot.Move(60, 1);
        robot.StrafeToWall(13, 1);
    }
}

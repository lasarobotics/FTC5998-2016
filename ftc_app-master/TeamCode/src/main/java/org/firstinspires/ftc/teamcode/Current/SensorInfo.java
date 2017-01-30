package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Ethan Schaffer on 10/6/2016.
 */
@Autonomous(name="Sensors", group="zSensor Testing")
public class SensorInfo extends LinearOpMode{
    Robot r = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {

        r.initialize(SensorInfo.this, hardwareMap, telemetry, true);
        while(!isStopRequested() && !isStarted()){
            r.sensorsInfo();
        }
        waitForStart();
        while(opModeIsActive()){
            r.sensorsInfo();
            idle();
        }
    }
}

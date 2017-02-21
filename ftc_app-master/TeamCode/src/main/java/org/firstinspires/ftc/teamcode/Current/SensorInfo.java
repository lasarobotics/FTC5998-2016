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

        r.initializeWithBotton(SensorInfo.this, hardwareMap, telemetry, true);
        while(!isStopRequested() && !isStarted()){
            r.Housekeeping();
        }
        waitForStart();
        while(opModeIsActive()){
            r.Housekeeping();
            idle();
        }
    }
}

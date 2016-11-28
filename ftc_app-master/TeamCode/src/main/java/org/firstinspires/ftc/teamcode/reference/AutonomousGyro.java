package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ethan Schaffer on 9/24/2016.
 */
@Autonomous(name = "MR: Gyro", group = "Autonomous")
//@Disabled   // comment out or remove this line to enable this opmode
public class AutonomousGyro extends LinearOpMode {
    ModernRoboticsI2cGyro gyroSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, "g");
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating()){
            telemetry.addData("Gyro Reading", "Calibrating...");
            telemetry.update();
        }
        telemetry.addData("Gyro Reading", "CALIBRATED");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Gyro Z", gyroSensor.getIntegratedZValue());
            telemetry.addData("Gyro Heading", gyroSensor.getHeading());
            telemetry.update();
            idle();
        }
    }
}

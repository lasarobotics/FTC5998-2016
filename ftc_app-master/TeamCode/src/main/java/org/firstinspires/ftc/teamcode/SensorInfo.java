package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Ethan Schaffer on 10/6/2016.
 */
@Autonomous(name="Sensors", group="zSensor Testing")
public class SensorInfo extends LinearOpMode{

    ColorSensor color_bottom, color_side;
    ModernRoboticsI2cRangeSensor range;
    @Override
    public void runOpMode() throws InterruptedException {

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r");
        color_bottom = hardwareMap.colorSensor.get("cb");
//        color_bottom = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "cb");
        color_bottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        color_side = hardwareMap.colorSensor.get("cs");
//        color_side = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "cs");
        color_side.setI2cAddress(I2cAddr.create8bit(0x3c));
        color_bottom.enableLed(true);
        color_side.enableLed(false);

        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("left red", color_bottom.red());
            telemetry.addData("left green", color_bottom.green());
            telemetry.addData("left blue", color_bottom.blue());
            telemetry.addData("left alpha", color_bottom.alpha());
            telemetry.addData("-----", "------");
            telemetry.addData("side red", color_side.red());
            telemetry.addData("side green", color_side.green());
            telemetry.addData("side blue", color_side.blue());
            telemetry.addData("--=--", "------");
            telemetry.addData("Distance", range.getDistance(DistanceUnit.CM));
            telemetry.addData("Optical", range.cmOptical());



            telemetry.update();
            idle();
        }
    }
}

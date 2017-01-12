package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Ethan Schaffer on 10/6/2016.
 */
@Autonomous(name="ColorSensor (3)", group="Autonomous")
@Disabled
public class colorSensorData extends LinearOpMode{

    ColorSensor color_bottom, color_bottom2, color_side;
    @Override
    public void runOpMode() throws InterruptedException {
        color_bottom = hardwareMap.colorSensor.get("cb");
        color_bottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        color_side = hardwareMap.colorSensor.get("cs");
        color_side.setI2cAddress(I2cAddr.create8bit(0x3c));
        color_bottom2 = hardwareMap.colorSensor.get("cb2");
        color_bottom2.setI2cAddress(I2cAddr.create8bit(0x2c));
        color_bottom.enableLed(true);
        color_bottom2.enableLed(true);
        color_side.enableLed(false);

        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("left red", color_bottom.red());
            telemetry.addData("left green", color_bottom.green());
            telemetry.addData("left blue", color_bottom.blue());
            telemetry.addData("left alpha", color_bottom.alpha());
            telemetry.addData("-----", "------");
            telemetry.addData("right red", color_bottom2.red());
            telemetry.addData("right green", color_bottom2.green());
            telemetry.addData("right blue", color_bottom2.blue());
            telemetry.addData("right alpha", color_bottom2.alpha());
            telemetry.addData("--=--", "--==--");
            telemetry.addData("side red", color_side.red());
            telemetry.addData("side green", color_side.green());
            telemetry.addData("side blue", color_side.blue());

            telemetry.update();
            idle();
        }
    }
}

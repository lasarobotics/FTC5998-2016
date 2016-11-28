package org.firstinspires.ftc.teamcode.reference;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.teamcode.navX.ftc.AHRS;

/**
 * Created by Ethan Schaffer on 9/24/2016.
 */
@Autonomous(name="navX", group="navX")
@Disabled
public class navXSimple extends LinearOpMode {
    AHRS navX;
    DeviceInterfaceModule dim;
    I2cDevice navXPortGetter;

    @Override
    public void runOpMode() throws InterruptedException {
        dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");
        navXPortGetter = hardwareMap.get(I2cDevice.class, "n");
        navX = new AHRS(dim, navXPortGetter.getPort(), AHRS.DeviceDataType.kProcessedData, (byte)50);
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Yaw", navX.getYaw());
            updateTelemetry(telemetry);
            idle();
        }
    }
}

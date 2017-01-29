package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.navX.ftc.AHRS;

import java.util.Arrays;
import java.util.Objects;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */

public class Robot {
    public enum team {
        Red, Blue, NotSensed
    }
    public static final String DIMNAME = "dim"; //second DIM, reserved for NavX
    public LinearOpMode l;
    private boolean infeedOn = false, shooterOn = false;
    public final double ticksPerRev = 7;
    public final double gearBoxOne = 40.0;
    public final double gearBoxTwo = 24.0 / 16.0;
    public final double gearBoxThree = 1.0;
    public final double wheelDiameter = 4.0 * Math.PI;
    public final double cmPerInch = 2.54;
    public final double width = 31.75;
    public final double ticksToStrafeDistance = 2000/(172*cmPerInch);
    //The Above Values lets us convert encoder ticks to centimeters per travelled, as shown below.

    public final double cmPerTick = (wheelDiameter / (ticksPerRev * gearBoxOne * gearBoxTwo * gearBoxThree)) * cmPerInch;
    //Allows us to drive our roobt with accuracy to the centiment

    public static final String LEFT1NAME = "l1"; //LX Port 2
    public static final String LEFT2NAME = "l2"; //LX Port 1
    public static final String RIGHT1NAME = "r1";//0A Port 1
    public static final String RIGHT2NAME = "r2";//0A Port 2
    public static final String BALLBLOCKLEFTNAME = "bl", BALLBLOCKRIGHTNAME = "br"; //MO Ports 3+4
    public static final double BALLBLOCKLEFTOPEN = 1, BALLBLOCKLEFTCLOSED = 0;
    public static final double BALLBLOCKRIGHTOPEN = 0, BALLBLOCKRIGHTCLOSED = 1;
    public static final String SHOOT1NAME = "sh1";//PN Port 1
    public static final String SHOOT2NAME = "sh2";//PN Port 2
    public static final String INFEEDNAME = "in"; //2S Port 2
    public static final String LIFTNAME = "l"; //2S Port 1
    public static final String LEFTPUSHNAME = "lp";//MO Port 1
    public static final String RIGHTPUSHNAME = "rp";//MO Port 2
    public static final String RANGENAME = "r"; //Port 0
    public static final String COLORSIDENAME = "cs"; //Port 1
    public static final String COLORLEFTBOTTOMNAME = "cb";//Port 2
    public static final String COLORRIGHTBOTTOMNAME = "cb2"; //Port 4
    public static final String GYRONAME = "g"; //Port 4
    public VoltageSensor voltageGetter;

    public DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed, lift;
    public Servo leftButtonPusher, rightButtonPusher, ballBlockRight, ballBlockLeft;
    public ColorSensor colorSensorOnSide, colorSensorBottom;
    public ModernRoboticsI2cGyro gyroSensor;
    public DeviceInterfaceModule dim;
    public ModernRoboticsI2cRangeSensor range;
    public AHRS navX;
    public static final double LEFT_SERVO_OFF_VALUE = .20;
    public static final double LEFT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_OFF_VALUE = .20;
    Telemetry t;
    public void sensorsInfo(){
        t.clear();
        t.addData("NavX", navX.isConnected() ? navX.getYaw() : "Disconnected");
        t.addData("Color Bottom", colorSensorBottom.alpha());
        t.addData("Color Side Red", colorSensorOnSide.red());
        t.addData("Color Side Blue", colorSensorOnSide.blue());
        t.addData("Range CM", range.getDistance(DistanceUnit.CM));
        t.update();
    }
    public void initialize(LinearOpMode lInput, HardwareMap hardwareMap, Telemetry telemetry, boolean navXOn){
        l = lInput;
        t = telemetry;
        leftFrontWheel = hardwareMap.dcMotor.get(LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(LEFT2NAME);
        rightFrontWheel = hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(RIGHT2NAME);
        rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        shoot1 = hardwareMap.dcMotor.get(SHOOT1NAME);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2 = hardwareMap.dcMotor.get(SHOOT2NAME);
        infeed = hardwareMap.dcMotor.get(INFEEDNAME);
        infeed.setDirection(DcMotorSimple.Direction.REVERSE);

        ballBlockRight = hardwareMap.servo.get(BALLBLOCKRIGHTNAME);
        ballBlockLeft = hardwareMap.servo.get(BALLBLOCKLEFTNAME);
        leftButtonPusher = hardwareMap.servo.get(LEFTPUSHNAME);
        rightButtonPusher = hardwareMap.servo.get(RIGHTPUSHNAME);

        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
        ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
        voltageGetter = hardwareMap.voltageSensor.get("Motor Controller 1");

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r");
//        colorSensorBottom = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "cb");
        colorSensorBottom = hardwareMap.colorSensor.get(COLORLEFTBOTTOMNAME);

        colorSensorBottom.setI2cAddress(I2cAddr.create8bit(0x4c));
//        colorSensorOnSide = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "cs");
        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorOnSide.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensorBottom.enableLed(true);
        colorSensorOnSide.enableLed(false);

        dim = hardwareMap.get(DeviceInterfaceModule.class, DIMNAME);
        //Based on the value of autoRouteChosen, we set the stateOrder array to the correct value
        navX = new AHRS(dim, hardwareMap.i2cDevice.get("n").getPort(), AHRS.DeviceDataType.kProcessedData, (byte)50);
        if(navXOn){
            navX.zeroYaw();
            while ( navX.isCalibrating() && !l.isStopRequested()) {
                telemetry.addData("Gyro", "Calibrating");
                telemetry.addData("Yaw", navX.getYaw());
                if(!navX.isConnected()){
                    telemetry.addData("NavX", "DISCONNECTED!");
                } else {
                    telemetry.addData("NavX", "Connected!");}
                telemetry.update();
            } //This silly looking code above animates a "..." sequence in telemetry, if the gyroscope is still calibrating
            telemetry.addData("Yaw", navX.getYaw());
            if(!navX.isConnected()){
                telemetry.addData("NavX", "DISCONNECTED!");
            } else {
                telemetry.addData("NavX", "Connected!");}
            telemetry.update();
        } else {
            telemetry.addData("NavX", "None");
        }

        colorSensorBottom.enableLed(true); //If you don't set the LED until after the waitForStart(), it doesn't work as well.
        colorSensorOnSide.enableLed(false);

    }

    public void setDrivePower(double power) {
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);
        rightBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
    }

    public void ResetDriveEncoders(){
        leftBackWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        while(leftFrontWheel.getCurrentPosition() != 0 || leftBackWheel.getCurrentPosition() != 0
                || rightBackWheel.getCurrentPosition() != 0 || rightFrontWheel.getCurrentPosition() != 0){

        }
        leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setStrafePower(String Direction, double Speed) {
        if(Objects.equals(Direction, "Left"))
        {
            rightFrontWheel.setPower(Speed);
            rightBackWheel.setPower(-Speed);
            leftFrontWheel.setPower(-Speed);
            leftBackWheel.setPower(Speed);
        }
        if(Objects.equals(Direction, "Right"))
        {
            rightFrontWheel.setPower(-Speed);
            rightBackWheel.setPower(Speed);
            leftFrontWheel.setPower(Speed);
            leftBackWheel.setPower(-Speed);
        }
    }

    public void AlignToWithin(double sensor, double power){
        TurnRightAbsolute(- sensor, power);
        TurnLeftAbsolute(sensor, power);
        TurnRightAbsolute(- sensor, power);
    }
    public void AlignToWithinOf(double expected, double threshold, double power){
        TurnRightAbsolute(expected - threshold, power);
        TurnLeftAbsolute(expected + threshold, power);
        TurnRightAbsolute(expected - threshold, power);
    }

    public void Move(double sensor, double power) {
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double ticks = sensor / cmPerTick;
        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
        double avg = (RBPos + LBPos + RFPos + LFPos)/4;
        while(avg < ticks && l.opModeIsActive()) {
            sensorsInfo();
            setDrivePower(power);
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        }
        setDrivePower(0);
    }
    public void MoveCoast(double sensor, double power) {
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double ticks = sensor / cmPerTick;
        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
        double avg = (RBPos + LBPos + RFPos + LFPos)/4;
        while(avg < ticks && l.opModeIsActive()) {
            sensorsInfo();
            setDrivePower(power);
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        }
    }

    public void ForwardsPLoop(double sensor, double maxPower) {
        //Max Power should be normally set to 1, but for very precise Movements a value of .25 or lower is reccomended.
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double ticks = sensor / cmPerTick;
        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
        double avg = (RBPos + LBPos + RFPos + LFPos)/4;
        double power;
        while(avg < ticks && l.opModeIsActive()) {
            sensorsInfo();
            power = Range.clip((ticks-avg)/ticks, .1, Math.abs(maxPower));
            setDrivePower(power);
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        }
        setDrivePower(0);
    }
    public void ForwardsPLoop(double sensor){
        ForwardsPLoop(sensor, 1.0);
    }
    public void BackwardsPLoop(double sensor, double maxPower) {
        //Max Power should be normally set to 1, but for very precise Movements a value of .25 or lower is reccomended.
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double ticks = Math.abs(sensor) / cmPerTick;
        int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
        int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
        int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
        int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
        double avg = (RBPos + LBPos + RFPos + LFPos)/4;
        double power;
        while(avg < ticks && l.opModeIsActive()) {
            sensorsInfo();
            power = - Range.clip((ticks-avg)/ticks, .1, Math.abs(maxPower));
            setDrivePower(power);
            RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + LBPos + RFPos + LFPos) / 4;
        }
        setDrivePower(0);
    }
    public void BackwardsPLoop(double sensor){
        BackwardsPLoop(sensor, 1.0);
    }
    public void TurnLeftPLoop(double degrees, double maxPower){
        //Max Power should be normally set to .5, but for very precise turns a value of .05 is reccomended.
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        double power;
        while(navX.getYaw() >= degrees && l.opModeIsActive()){
            sensorsInfo();
            power = Range.clip((navX.getYaw() - degrees)/degrees, .05, maxPower);
            rightBackWheel.setPower(power);
            rightFrontWheel.setPower(power);
            leftBackWheel.setPower(-power);
            leftFrontWheel.setPower(-power);
        }
        setDrivePower(0);
    }

    public void TurnRightPLoop(double degrees, double maxPower){
        //Max Power should be normally set to .5, but for very precise turns a value of .05 is reccomended.
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        double power;
        while(navX.getYaw() <= degrees && l.opModeIsActive()){
            sensorsInfo();
            power = Range.clip((degrees - navX.getYaw())/degrees, .05, maxPower);
            rightBackWheel.setPower(power);
            rightFrontWheel.setPower(power);
            leftBackWheel.setPower(-power);
            leftFrontWheel.setPower(-power);
        }
        setDrivePower(0);
    }


    public void TurnLeftAbsolute(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        while(navX.getYaw() >= sensor && l.opModeIsActive()){
            sensorsInfo();
            rightBackWheel.setPower(power);
            rightFrontWheel.setPower(power);
            leftBackWheel.setPower(-power);
            leftFrontWheel.setPower(-power);
        }
        setDrivePower(0);

    }
    public void TurnLeftRelative(double sensor, double power){
        if(!navX.isConnected()){
            TurnLeftEnc(sensor, .10);
            return;
        }
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        double yaw = navX.getYaw();
        sensor = -Math.abs(sensor);
        while(Math.abs(yaw - navX.getYaw()) < sensor && l.opModeIsActive()){
            sensorsInfo();
            rightBackWheel.setPower(power);
            rightFrontWheel.setPower(power);
            leftBackWheel.setPower(-power);
            leftFrontWheel.setPower(-power);
        }
        setDrivePower(0);

    }
    public void TurnLeft(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        sensor = - Math.abs(sensor);
        while(navX.getYaw() >= sensor && l.opModeIsActive()){
            sensorsInfo();
            rightBackWheel.setPower(power);
            rightFrontWheel.setPower(power);
            leftBackWheel.setPower(-power);
            leftFrontWheel.setPower(-power);
        }
        setDrivePower(0);

    }

    public void TurnLeftEnc(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double changeFactor = 90;
        double modified = changeFactor + sensor;

        double circleFrac = modified/360;
        double cm = width * circleFrac * Math.PI * 2;
        double movement = cm / cmPerTick;
        double ticks = movement/2;

        rightBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
        leftBackWheel.setPower(-power);
        leftFrontWheel.setPower(-power);

        double avg = 0;
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos)/4;
            sensorsInfo();
        } while(avg < ticks && l.opModeIsActive());
        setDrivePower(0);
    }

    public void TurnRightAbsolute(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        while(navX.getYaw() <= sensor && l.opModeIsActive()){
            sensorsInfo();
            rightBackWheel.setPower(-power);
            rightFrontWheel.setPower(-power);
            leftBackWheel.setPower(power);
            leftFrontWheel.setPower(power);
        }
        setDrivePower(0);
    }
    public void TurnRight(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        sensor = Math.abs(sensor);
        while(navX.getYaw() <= sensor && l.opModeIsActive()){
            sensorsInfo();
            rightBackWheel.setPower(-power);
            rightFrontWheel.setPower(-power);
            leftBackWheel.setPower(power);
            leftFrontWheel.setPower(power);
        }
        setDrivePower(0);
    }
    public void TurnRightRelative(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        double yaw = navX.getYaw();
        while(Math.abs(yaw - navX.getYaw()) < sensor && l.opModeIsActive()){
            sensorsInfo();
            rightBackWheel.setPower(-power);
            rightFrontWheel.setPower(-power);
            leftBackWheel.setPower(power);
            leftFrontWheel.setPower(power);
        }
        setDrivePower(0);
    }

    public void TurnRightEnc(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        ResetDriveEncoders();
        double changeFactor = 90;
        double modified = changeFactor + sensor;

        double circleFrac = modified/360;
        double cm = width * circleFrac * Math.PI * 2;
        double movement = cm / cmPerTick;
        double ticks = movement/2;

        rightBackWheel.setPower(-power);
        rightFrontWheel.setPower(-power);
        leftBackWheel.setPower(power);
        leftFrontWheel.setPower(power);

        double avg = 0;
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos)/4;
            sensorsInfo();
        } while(avg < ticks && l.opModeIsActive());
        setDrivePower(0);
    }
    public void StrafeLeft(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        double ticks = sensor / cmPerTick;
        ticks *= ticksToStrafeDistance;
        int avg = 0;
        rightFrontWheel.setPower(power);
        rightBackWheel.setPower(-power);
        leftFrontWheel.setPower(-power);
        leftBackWheel.setPower(power);
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos)/4;
            sensorsInfo();
        } while(avg < ticks && l.opModeIsActive());
        setDrivePower(0);
    }
    public void StrafeRight(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        double ticks = sensor / cmPerTick;
        ticks *= ticksToStrafeDistance;
        int avg = 0;
        setStrafePower("Right", power);
        do {
            int RBPos = Math.abs(rightBackWheel.getCurrentPosition());
            int RFPos = Math.abs(rightFrontWheel.getCurrentPosition());
            int LBPos = Math.abs(leftBackWheel.getCurrentPosition());
            int LFPos = Math.abs(leftFrontWheel.getCurrentPosition());
            avg = (RBPos + RFPos + LBPos + LFPos)/4;
            sensorsInfo();
        } while(avg < ticks && l.opModeIsActive());
        setDrivePower(0);
    }
    public double getRange(double previous){
        double c = range.getDistance(DistanceUnit.CM);
        if(c == 255){
            return previous;
        } else {
            return c;
        }
    }
    public void ShootSmart(){
        shooterOn = true;
        double volts = voltageGetter.getVoltage();
        double power = 1.00;
        if(volts > 13.3){
            power = 0.40;
        } else if(volts > 13.1){
            power = 0.50;
        } else if(volts > 12.9){
            power = 0.55;
        } else if(volts > 12.6){
            power = 0.60;
        } else if(volts > 12.3) {
            power = 0.70;
        } else if(volts > 12.0) {
            power = 0.80;
        } else {
            power = 0.90;
        }
        shoot1.setPower(power);
        shoot2.setPower(power);
    }
    public void StopShooter(){
        shoot1.setPower(0);
        shoot2.setPower(0);
        shooterOn = false;
    }
    public void StrafeToWall(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        double pastRange = 254;
        if(!l.opModeIsActive())
            Finish();
        while(pastRange > sensor && l.opModeIsActive()){
            sensorsInfo();
            pastRange = getRange(pastRange);
            setStrafePower("Left", power);
        }
        if(range.getDistance(DistanceUnit.CM) == 255){
            StrafeToWall(sensor, power);
        }
        setDrivePower(0);
    }
    public void StrafeFromWall(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        double pastRange = 0;
        if(!l.opModeIsActive())
            Finish();
        while(pastRange < sensor && l.opModeIsActive()){
            sensorsInfo();
            pastRange = getRange(pastRange);
            setStrafePower("Right", power);
        }
        if(range.getDistance(DistanceUnit.CM) == 255){
            StrafeFromWall(sensor, power);
        }
        setDrivePower(0);
    }

    public void LineSearch(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        if(!l.opModeIsActive())
            Finish();
        while(colorSensorBottom.red() < sensor && colorSensorBottom.blue() < sensor && colorSensorBottom.alpha() < sensor && l.opModeIsActive()){
            sensorsInfo();
            setDrivePower(power);
        }
        setDrivePower(0);
    }
    public void StrafeToPrecise(double cm, double threshold, double power){
        StrafeToWall(cm+threshold, power);
        StrafeFromWall(cm-threshold, power);
        StrafeToWall(cm+threshold, power);
    }
    public void PressBeaconSimple(team t){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        team colorReading;
        if(colorSensorOnSide.red() == 255){
            return;
        }
        if(colorSensorOnSide.red() > colorSensorOnSide.blue()){
            colorReading = team.Red;
        } else {
            colorReading = team.Blue;
        }
        if(colorReading == t){
            Move(1, - .25);
            rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        } else {
            rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
        }
        try {
            Thread.sleep(1250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
    }

    public void PressBeacon(team t){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        team colorReading;
        if(colorSensorOnSide.red() == 255){
            return;
        }
        if(colorSensorOnSide.red() > colorSensorOnSide.blue()){
            colorReading = team.Red;
        } else {
            colorReading = team.Blue;
        }
        if(colorReading == t){
            Move(1, - .25);
            rightButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        } else {
            rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
        }
            try {
                Thread.sleep(1250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        try {
            Thread.sleep(1250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        if(!l.opModeIsActive())
        {
            Finish();
        }
        if(colorSensorOnSide.red() > colorSensorOnSide.blue()){
            colorReading = team.Red;
        } else {
            colorReading = team.Blue;
        }
        if(colorReading == t){
            return;
        } else {
            try {
                Thread.sleep(150);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if(colorSensorOnSide.red() > colorSensorOnSide.blue()){
                colorReading = team.Red;
            } else {
                colorReading = team.Blue;
            }
            if(colorReading == t){
                rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
                leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
                return;
            }
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
            try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
            leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        }

    }
    public void ShootAtPower(double power){
        shooterOn = true;
        if(!l.opModeIsActive())
            Finish();
        shoot1.setPower(power);
        shoot2.setPower(power);
    }
    public void EnableShot(double sensor, double power){
        infeedOn = true;
        if(!l.opModeIsActive())
            Finish();
        ballBlockLeft.setPosition(BALLBLOCKLEFTOPEN);
        ballBlockRight.setPosition(BALLBLOCKRIGHTOPEN);
        infeed.setPower(power);
        infeed.setPower(power);
        try{
            Thread.sleep((long)sensor);
        } catch (InterruptedException e){
            e.printStackTrace();
        }
        ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
        ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
        infeed.setPower(0);
        infeedOn = false;
    }
    public void Finish(){
        navX.close();
        colorSensorOnSide.close();
        colorSensorBottom.close();
        range.close();
        l.stop();
    }

    public void DiagonalForwardsLeft(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        double pastRange = 254;
        if(!l.opModeIsActive())
            Finish();
        while(pastRange > sensor && l.opModeIsActive()){
            sensorsInfo();
            pastRange = getRange(pastRange);
            arcadeMecanum(1, -1, 0);
        }
        setDrivePower(0);
    }

    public void DiagonalForwardsRight(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        double pastRange = 254;
        if(!l.opModeIsActive())
            Finish();
        while(pastRange > sensor && l.opModeIsActive()){
            sensorsInfo();
            pastRange = getRange(pastRange);
            arcadeMecanum(power, power, 0);
        }
        setDrivePower(0);
    }

    public void DiagonalBackwardsRight(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        double pastRange = 254;
        if(!l.opModeIsActive())
            Finish();
        while(pastRange > sensor && l.opModeIsActive()){
            sensorsInfo();
            pastRange = getRange(pastRange);
            arcadeMecanum(-power, power, 0);
        }
        setDrivePower(0);
    }
    public void DiagonalBackwardsRightCoast(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        double pastRange = 254;
        if(!l.opModeIsActive())
            Finish();
        while(pastRange > sensor && l.opModeIsActive()){
            sensorsInfo();
            pastRange = getRange(pastRange);
            arcadeMecanum(-power, power, 0);
        }
    }
    public void DiagonalBackwardsLeftCoast(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        double pastRange = 254;
        if(!l.opModeIsActive())
            Finish();
        while(pastRange > sensor && l.opModeIsActive()){
            sensorsInfo();
            pastRange = getRange(pastRange);
            arcadeMecanum(-power, -power, 0);
        }
    }

    public void DiagonalBackwardsLeft(double sensor, double power){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        double pastRange = 254;
        if(!l.opModeIsActive())
            Finish();
        while(pastRange > sensor && l.opModeIsActive()){
            sensorsInfo();
            pastRange = getRange(pastRange);
            arcadeMecanum(-power, -power, 0);
        }
        setDrivePower(0);
    }
    public void DiagonalBackwardsLeft(double sensor, double yIn, double xIn){
        if(infeedOn){
            infeed.setPower(1);
        } else {
            infeed.setPower(0);
        }
        if(shooterOn){
            ShootSmart();
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }
        double pastRange = 254;
        if(!l.opModeIsActive())
            Finish();
        while(pastRange > sensor && l.opModeIsActive()){
            sensorsInfo();
            pastRange = getRange(pastRange);
            arcadeMecanum(-yIn, -xIn, 0);
        }
        setDrivePower(0);
    }

    public void arcadeMecanum(double y, double x, double c) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between -1 and +1, if not there already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        leftFrontWheel.setPower(leftFrontVal);
        leftBackWheel.setPower(leftBackVal);
        rightFrontWheel.setPower(rightFrontVal);
        rightBackWheel.setPower(rightBackVal);
    }

}

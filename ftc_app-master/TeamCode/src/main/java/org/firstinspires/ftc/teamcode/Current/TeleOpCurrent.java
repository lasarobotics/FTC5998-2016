/*
ADB guide can be found at:
https://ftcprogramming.wordpress.com/2015/11/30/building-ftc_app-wirelessly/
*/
package org.firstinspires.ftc.teamcode.Current;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

/**
 * Created by Ethan Schaffer on 10/31/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele Op", group="TeleOp")
public class TeleOpCurrent extends OpMode {
    public double rpmTarget = 2400;
    public static final double LEFT_SERVO_OFF_VALUE = .3;
    public static final double LEFT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_ON_VALUE = 1;
    public static final double RIGHT_SERVO_OFF_VALUE = .3;

    public static final double MAXINFEEDPOWER = 1;
    public static final double MAXOUTFEEDPOWER = -1;

    //GOOD VALUES
    public enum INFEEDSTATE {
        IN, OUT, NOTGOING
    }
    public enum SHOOTERSTATE {
        SHOOTING, BACK, NOTSHOOTING
    }
    public enum SERVOSTATE {
        ON, OFF
    }
    public INFEEDSTATE INFEEDSTATUS = INFEEDSTATE.NOTGOING;
    public SHOOTERSTATE SHOOTERSTATUS = SHOOTERSTATE.NOTSHOOTING;
    public SERVOSTATE RIGHTSERVOSTATE = SERVOSTATE.OFF;
    public SERVOSTATE LEFTSERVOSTATE = SERVOSTATE.OFF;

    public boolean RECENT_DRIVER_RIGHT_BUMPER = false;
    public boolean RECENT_Y_BUTTON = false;
    public boolean RECENT_TRIGGER = false;

    public boolean RECENT_X_BUTTON = false;
    public boolean RECENT_B_BUTTON = false;
    public boolean RECENT_TOPHAT_DOWN = false;
    public boolean RECENT_TOPHAT_UP = false;

    public boolean RECENT_LB = false;
    public boolean RECENT_RB = false;


    public static final double SLOWDOWNVALUE = 5;
    public static final double TRIGGERTHRESHOLD = .2;
    public static final double ACCEPTINPUTTHRESHOLD = .15;
    public static final double SCALEDPOWER = 1; //Emphasis on current controller reading (vs current motor power) on the drive train

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
    final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);

    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, shoot1, shoot2, infeed, lift;
    Servo leftButtonPusher, rightButtonPusher, ballBlockRight, ballBlockLeft;
    ColorSensor colorSensorOnSide, colorSensorLeftBottom, colorSensorRightBottom;
    ModernRoboticsI2cGyro gyroSensor;
    public double volts;
    public double rpm1;
    public double power;
    public double lastTime, lastEnc1, lastEnc2;
    public double rpm2;
    DeviceInterfaceModule dim;
    ModernRoboticsI2cRangeSensor range;
    public double SHOOTERMAXVALUE = 1;

    @Override
    public void init() {
        leftFrontWheel = hardwareMap.dcMotor.get(LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(LEFT2NAME);
        rightFrontWheel = hardwareMap.dcMotor.get(RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(RIGHT2NAME);
        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hardwareMap.dcMotor.get(LIFTNAME);

        shoot1 = hardwareMap.dcMotor.get(SHOOT1NAME);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2 = hardwareMap.dcMotor.get(SHOOT2NAME);
        infeed = hardwareMap.dcMotor.get(INFEEDNAME);
        infeed.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //allows the shooter to slow down properly
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //to avoid the gearboxes getting damaged over time
        ballBlockRight = hardwareMap.servo.get(BALLBLOCKRIGHTNAME);
        ballBlockLeft = hardwareMap.servo.get(BALLBLOCKLEFTNAME);
        leftButtonPusher = hardwareMap.servo.get(LEFTPUSHNAME);
        rightButtonPusher = hardwareMap.servo.get(RIGHTPUSHNAME);

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGENAME);
        colorSensorLeftBottom = hardwareMap.colorSensor.get(COLORLEFTBOTTOMNAME);
        colorSensorRightBottom = hardwareMap.colorSensor.get(COLORRIGHTBOTTOMNAME);
        colorSensorOnSide = hardwareMap.colorSensor.get(COLORSIDENAME);
        colorSensorLeftBottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        colorSensorOnSide.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensorRightBottom.setI2cAddress(I2cAddr.create8bit(0x2c));
        dim = hardwareMap.get(DeviceInterfaceModule.class, "Device Interface Module 1");
        telemetry.addData("raw ultrasonic", range.rawUltrasonic());
        telemetry.update();

        leftButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        rightButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
        ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
        lastEnc2 = 0;
        volts = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        power = 1.0;
    }

    @Override
    public void loop() {

        /*
              _                 _
             | |               | |
          ___| |__   ___   ___ | |_
         / __| '_ \ / _ \ / _ \| __|
         \__ \ | | | (_) | (_) | |_
         |___/_| |_|\___/ \___/ \__|
        */
        if(!RECENT_LB && gamepad2.left_bumper){
            SHOOTERSTATUS =(SHOOTERSTATUS == SHOOTERSTATE.SHOOTING) ? SHOOTERSTATE.NOTSHOOTING : SHOOTERSTATE.SHOOTING;
        } else if (!RECENT_RB && gamepad2.right_bumper){
            SHOOTERSTATUS =(SHOOTERSTATUS == SHOOTERSTATE.BACK) ? SHOOTERSTATE.NOTSHOOTING : SHOOTERSTATE.BACK;
        }
        //Ternary Operations used to toggle SHOOTERSTATUS
        switch(SHOOTERSTATUS){
            case SHOOTING:
                volts = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
                rpm1 = (Math.abs((shoot1.getCurrentPosition()- lastEnc1)) / (7*4) ) //Rotations
                            / // Per
                        ( (System.nanoTime()-lastTime)/(NANOSECONDS_PER_SECOND*60) ); // Minute
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
                if(rpm1 != 0){
                    if(rpm1 < rpmTarget - 1200){
                        shoot1.setPower(power);
                        shoot2.setPower(power);
                        telemetry.addData("RPM1", "Just Started");
                    } else if(rpm1 > rpmTarget+100){
                        shoot1.setPower(shoot1.getPower() + .01);
                        shoot2.setPower(shoot1.getPower() + .01);
                        telemetry.addData("RPM1", "Low");
                    } else if(rpm1 < rpmTarget-100){
                        shoot1.setPower(shoot1.getPower() - .01);
                        shoot2.setPower(shoot1.getPower() - .01);
                        telemetry.addData("RPM1", "High");
                    } else {
                        telemetry.addData("RPM1", "Good");
                    }
                    telemetry.clear();
                    telemetry.addData("RPM1 Value", rpm1);
                    telemetry.addData("RPM2 Value", rpm2);
                    telemetry.addData("RPM Target", rpmTarget);
                    telemetry.addData("Power by RPM1", shoot1.getPower());
                    telemetry.addData("Power by RPM2", shoot2.getPower());
                    telemetry.addData("Power by volts", power);
                    telemetry.update();
                    ballBlockRight.setPosition(BALLBLOCKRIGHTOPEN);
                    ballBlockLeft.setPosition(BALLBLOCKLEFTOPEN);

                    lastEnc1 = shoot1.getCurrentPosition();
                    lastTime = System.nanoTime();
                } else {
                    telemetry.clear();
                    telemetry.addData("RPM", "At Zero");
                    telemetry.update();
                }
                break;
            case BACK:
                shoot1.setPower(-.75);
                shoot2.setPower(-.75);
                ballBlockRight.setPosition(BALLBLOCKRIGHTOPEN);
                ballBlockLeft.setPosition(BALLBLOCKLEFTOPEN);
                break;
            default:
                shoot1.setPower(0);
                shoot2.setPower(0);
                ballBlockRight.setPosition(BALLBLOCKRIGHTCLOSED);
                ballBlockLeft.setPosition(BALLBLOCKLEFTCLOSED);
        }
        RECENT_LB = gamepad2.left_bumper;
        RECENT_RB = gamepad2.right_bumper;
        /*
          _        __              _
         (_)      / _|            | |
          _ _ __ | |_ ___  ___  __| |
         | | '_ \|  _/ _ \/ _ \/ _` |
         | | | | | ||  __/  __/ (_| |
         |_|_| |_|_| \___|\___|\__,_|
        */
        if(gamepad2.dpad_down){
            infeed.setPower(MAXINFEEDPOWER);
        } else if(gamepad2.dpad_up) {
            infeed.setPower(MAXOUTFEEDPOWER);
        } else {
            infeed.setPower(0);
        }
        /*
              _      _
             | |    (_)
           __| |_ __ ___   _____
          / _` | '__| \ \ / / _ \
         | (_| | |  | |\ V /  __/
          \__,_|_|  |_| \_/ \___|
         */
        double inputY = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_y : 0;
        double inputX = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_x : 0;
        double inputC = Math.abs(gamepad1.right_stick_x)> ACCEPTINPUTTHRESHOLD ? -gamepad1.right_stick_x: 0;

        double BIGGERTRIGGER = gamepad1.left_trigger > gamepad1.right_trigger ? gamepad1.left_trigger : gamepad1.right_trigger;
        //Ternary, the larger trigger value is set to the value BIGGERTRIGGER

        if(BIGGERTRIGGER > TRIGGERTHRESHOLD){ //If we have enough pressure on a trigger
            if( (Math.abs(inputY) > Math.abs(inputX)) && (Math.abs(inputY) > Math.abs(inputC)) ){ //If our forwards motion is the largest motion vector
                inputY /= 5*BIGGERTRIGGER; //slow down our power inputs
                inputX /= 5*BIGGERTRIGGER; //slow down our power inputs
                inputC /= 5*BIGGERTRIGGER; //slow down our power inputs
            } else if( (Math.abs(inputC) > Math.abs(inputX)) && (Math.abs(inputC) > Math.abs(inputY)) ){ //and if our turing motion is the largest motion vector
                inputY /= 4*BIGGERTRIGGER; //slow down our power inputs
                inputX /= 4*BIGGERTRIGGER; //slow down our power inputs
                inputC /= 4*BIGGERTRIGGER; //slow down our power inputs
            } else if( (Math.abs(inputX) > Math.abs(inputY)) && (Math.abs(inputX) > Math.abs(inputC)) ){ //and if our strafing motion is the largest motion vector
                inputY /= 3*BIGGERTRIGGER; //slow down our power inputs
                inputX /= 3*BIGGERTRIGGER; //slow down our power inputs
                inputC /= 3*BIGGERTRIGGER; //slow down our power inputs
            }
        }
        //Use the larger trigger value to scale down the inputs.

        arcadeMecanum(inputY, inputX, inputC, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

        /*
          ___  ___ _ ____   _____
         / __|/ _ \ '__\ \ / / _ \
         \__ \  __/ |   \ V / (_) |
         |___/\___|_|    \_/ \___/
         */

        if(RECENT_X_BUTTON && !gamepad2.x){
            LEFTSERVOSTATE = (LEFTSERVOSTATE == SERVOSTATE.OFF ? SERVOSTATE.ON : SERVOSTATE.OFF);
        }
        switch (RIGHTSERVOSTATE){
            case ON:
                leftButtonPusher.setPosition(RIGHT_SERVO_ON_VALUE);
                break;
            case OFF:
                leftButtonPusher.setPosition(RIGHT_SERVO_OFF_VALUE);
        }
        RECENT_X_BUTTON = gamepad2.x;

        if(RECENT_B_BUTTON && !gamepad2.b){
            RIGHTSERVOSTATE = (RIGHTSERVOSTATE== SERVOSTATE.OFF ? SERVOSTATE.ON : SERVOSTATE.OFF);
        }
        switch (LEFTSERVOSTATE){
            case ON:
                rightButtonPusher.setPosition(LEFT_SERVO_ON_VALUE);
                break;
            case OFF:
                rightButtonPusher.setPosition(LEFT_SERVO_OFF_VALUE);
        }
        RECENT_B_BUTTON = gamepad2.b;

    }

    // y - forwards
    // x - side
    // c - rotation
    public static void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        double scaledPower = SCALEDPOWER;

        leftFront.setPower(leftFrontVal*scaledPower+leftFront.getPower()*(1-scaledPower));
        rightFront.setPower(rightFrontVal*scaledPower+rightFront.getPower()*(1-scaledPower));
        leftBack.setPower(leftBackVal*scaledPower+leftBack.getPower()*(1-scaledPower));
        rightBack.setPower(rightBackVal*scaledPower+rightBack.getPower()*(1-scaledPower));
    }
}

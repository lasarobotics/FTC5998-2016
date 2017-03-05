package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Ethan Schaffer on 1/25/2017.
 */
@Autonomous(name = "R 115", group = "New")
public class _Red115 extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        String LEFTPUSHNAME = "lp";//MO Port 1
        String RIGHTPUSHNAME = "rp";//MO Port 2
        Servo leftButtonPusher = hardwareMap.servo.get(LEFTPUSHNAME);
        Servo rightButtonPusher = hardwareMap.servo.get(RIGHTPUSHNAME);
        robot.initialize(_Red115.this, hardwareMap, telemetry, true);
        while (!isStarted() && !isStopRequested()) {
            robot.sensorsInfo();
        }
        waitForStart(); //Should be unecessary, as isStarted() is only true when the start button is hit
        robot.infeed.setPower(1);
        sleep(250);
        robot.infeed.setPower(0);
        robot.Move(25, -1);
        robot.ArcadeToAngleLeft(0, .25, -.40, 20);
        robot.ShootByVoltage();
        robot.AlignToWithinOf(-30, .5, .05);

        robot.Move(150, 1);
        robot.AlignToWithinOf(70, 5, .15);
        robot.AlignToWithinOf(90, 1.5, .05);
        robot.EnableShot();
        robot.StopShooter();
        robot.AlignToWithin(1.5, .05);
        robot.DiagonalForwardsLeft(9, .65, 1);
        robot.AlignToWithinOf(-2, .5, .05);
        robot.FindAndPressSquareToBeacon(Robot.team.Red, -.10);
        leftButtonPusher.setPosition(.3);
        rightButtonPusher.setPosition(.3);
        robot.CheckBeacon(Robot.team.Red);
        leftButtonPusher.setPosition(.3);
        rightButtonPusher.setPosition(.3);

        robot.Move(85, 1.0);
        robot.StrafeFromWall(9, .10);
        robot.AlignToWithinOf(2, .5, .05);
        robot.FindAndPressSquareToBeacon(Robot.team.Red, .10);
        leftButtonPusher.setPosition(.3);
        rightButtonPusher.setPosition(.3);
        robot.CheckBeacon(Robot.team.Red);
        leftButtonPusher.setPosition(.3);
        rightButtonPusher.setPosition(.3);

        robot.ArcadeToAngleLeft(0, .25, -.40, 20);
        robot.AlignToWithinOf(-40, 1, .05);
        robot.Move(180, -1.0);
    }
}

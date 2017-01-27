package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

/**
 * Created by Ethan Schaffer on 1/11/2017.
 */
@Autonomous(group = "Red", name = "R_2B")
public class ReadInput extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(ReadInput.this, hardwareMap, telemetry, true);
        while (!isStarted()) {
            robot.sensorsInfo();
        }
        waitForStart();
        String FILE_DIR = "/LASA";
        File file = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + FILE_DIR, "route.txt");
        Scanner s = null;
        try {
            s = new Scanner(file);
            while (s.hasNextDouble()) {
                double functionVal;
                functionVal = s.nextDouble();
                switch ((int)functionVal){
                    case 1:
                        robot.Move(s.nextDouble(), s.nextDouble());
                        break;
                    case 2:
                        robot.TurnLeft(s.nextDouble(), s.nextDouble());
                        break;
                    case 3:
                        robot.TurnRight(s.nextDouble(), s.nextDouble());
                        break;
                    case 4:
                        robot.StrafeFromWall(s.nextDouble(), s.nextDouble());
                        break;
                    case 5:
                        double color = s.nextDouble();
                        robot.PressBeacon(color == 1 ? Robot.team.Blue : Robot.team.Red);
                        break;
                    case 6:
                        double color_2 = s.nextDouble();
                        robot.PressBeaconSimple(color_2 == 1 ? Robot.team.Blue : Robot.team.Red);
                        break;
                    case 7:
                        double color_3 = s.nextDouble();
                        robot.PressBeaconSmart(color_3 == 1 ? Robot.team.Blue : Robot.team.Red);
                        break;
                    case 8:
                        robot.ShootSmart();
                        break;
                    case 9:
                        robot.StopShooter();
                        break;
                    case 10:
                        sleep((long)s.nextDouble());
                        break;
                    case 11:
                        robot.DiagonalForwardsLeft(s.nextDouble(), s.nextDouble());
                        break;
                    case 12:
                        robot.DiagonalForwardsRight(s.nextDouble(), s.nextDouble());
                        break;
                    case 13:
                        robot.DiagonalBackwardsLeft(s.nextDouble(), s.nextDouble());
                        break;
                    case 14:
                        robot.DiagonalBackwardsRight(s.nextDouble(), s.nextDouble());
                        break;
                    case 15:
                        robot.AlignToWithin(s.nextDouble(), s.nextDouble());
                        break;
                    case 16:
                        robot.AlignToWithinOf(s.nextDouble(), s.nextDouble(), s.nextDouble());
                        break;
                    case 17:
                        robot.EnableShot(s.nextDouble(), s.nextDouble());
                        break;
                    case 18:
                        robot.LineSearch(s.nextDouble(), s.nextDouble());
                        break;
                    case 19:
                        robot.StrafeToWall(s.nextDouble(), s.nextDouble());
                        break;
                    case 20:
                        robot.ForwardsPLoop(s.nextDouble(), s.nextDouble());
                        break;
                    case 21:
                        robot.TurnLeftEnc(s.nextDouble(), s.nextDouble());
                        break;
                    case 22:
                        robot.TurnRightEnc(s.nextDouble(), s.nextDouble());
                        break;
                    case 23:
                        robot.StrafeToPrecise(s.nextDouble(), s.nextDouble(), s.nextDouble());
                        break;
                    case 24:
                        robot.TurnRightRelative(s.nextDouble(), s.nextDouble());
                        break;
                    case 25:
                        robot.TurnLeftRelative(s.nextDouble(), s.nextDouble());
                        break;
                    case 26:
                        robot.TurnLeftAbsolute(s.nextDouble(), s.nextDouble());
                        break;
                    case 27:
                        robot.TurnRightAbsolute(s.nextDouble(), s.nextDouble());
                        break;
                    default:
                        robot.Finish();
                        break;
                }
            }
        } catch (FileNotFoundException e){
            e.printStackTrace();
        }

    }
}
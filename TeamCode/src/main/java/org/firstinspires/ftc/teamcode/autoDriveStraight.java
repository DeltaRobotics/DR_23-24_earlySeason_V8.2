package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="autoDriveStraight")
//@Disabled
public class autoDriveStraight extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        //telemetry.addData("hello",1.5);
        telemetry.addData("heading", robot.GlobalHeading);
        telemetry.addData("90", 90);
        telemetry.addData("value",0);
        telemetry.update();

        waitForStart();

        robot.wait(2000, robot.odometers);

        double y = 0;
        double x = 0;
        double finalAngle = Math.toRadians(180);

        while(Math.abs(robot.angleWrapRad(finalAngle - robot.GlobalHeading)) > robot.angleAccuracy) {
            //robot.goToPosSingle(20, 00, 00, 0);
            //telemetry.addData("hello world",x);


            telemetry.addData("value",robot.goToPosSingle(x, y, finalAngle, 0));

            telemetry.addData("pid 90",robot.odoTurnPID(0,Math.toRadians(90)));

            telemetry.addData("globalY",robot.GlobalY);




            telemetry.addData("heading", robot.GlobalHeading);
            telemetry.addData("90", 2);
            telemetry.update();
        }

        //telemetry.addData("deg",robot.GlobalHeading * 57.295);
        //telemetry.update();

        //robot.wait(2000, robot.odometers);

    }
}

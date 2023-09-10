package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="driveWithOdo")
//@Disabled

public class driveWithOdo extends LinearOpMode{

    public void runOpMode() throws InterruptedException {

        robotHardware robot = new robotHardware(hardwareMap);

        robot.resetDriveEncoders();

        robot.wrist.setPosition(0);
        robot.finger.setPosition(0);

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 0.75);

            if (gamepad1.a){
                robot.arm.setTargetPosition(0);
                robot.arm.setPower(.25);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.b){
                robot.arm.setTargetPosition(100);
                robot.arm.setPower(.25);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            //telemetry.addData("motorRFPower", robot.motorRF.getPower());
            //telemetry.addData("motorRBPower", robot.motorRB.getPower());
            //telemetry.addData("motorLBPower", robot.motorLB.getPower());
            //telemetry.addData("motorLFPower", robot.motorLF.getPower());

            robot.refresh(robot.odometers);

            //telemetry.addData("left encoder", -robot.leftEncoder.getCurrentPosition());
            //telemetry.addData("right encoder", -robot.rightEncoder.getCurrentPosition());
            //telemetry.addData("perpendicular encoder", robot.perpendicularEncoder.getCurrentPosition());

            telemetry.addData("x", robot.GlobalX);
            telemetry.addData("y", robot.GlobalY);
            telemetry.addData("heading", robot.GlobalHeading);

            telemetry.addData("perp", robot.odometers[2].getCurrentPosition());

            telemetry.addData("rotations", robot.GlobalHeading * 57.295 / 360);
            telemetry.addData("deg", robot.GlobalHeading * 57.295);

            telemetry.update();
        }
    }
}
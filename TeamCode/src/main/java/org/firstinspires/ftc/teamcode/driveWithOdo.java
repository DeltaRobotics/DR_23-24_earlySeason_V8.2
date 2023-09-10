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
        robot.launcher.setPosition(1);

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 0.75);

            if (gamepad1.x){
                //zero position
                robot.wrist.setPosition(0);
                robot.arm.setTargetPosition(0);
                robot.arm.setPower(.35);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.b){
                //placing
                robot.arm.setTargetPosition(800);
                robot.arm.setPower(.35);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.a && robot.arm.getCurrentPosition() > 750){
                //collecting
                robot.arm.setTargetPosition(1225);
                robot.arm.setPower(.35);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.right_bumper){
                //grab
                robot.finger.setPosition(0.5);
            }
            if(gamepad1.left_bumper){
                //release
                robot.finger.setPosition(0);
            }

            if(gamepad1.dpad_up){
                //slight change
                robot.wrist.setPosition(robot.wrist.getPosition() + .05);
            }
            if(gamepad1.dpad_down){
                robot.wrist.setPosition(robot.wrist.getPosition() - .05);
            }

            robot.refresh(robot.odometers);

            if(robot.arm.getCurrentPosition()<600){
                robot.wrist.setPosition(0);
            } else if(robot.arm.getCurrentPosition() > 600 && robot.arm.getCurrentPosition() < 850){
                robot.wrist.setPosition(.7);
            } else if(robot.arm.getCurrentPosition() > 850){
                robot.wrist.setPosition(.83);
            }

            if(gamepad2.a && gamepad2.b){
                robot.launcher.setPosition(0);
            }

            //keep for testing
            if(gamepad2.x){
                robot.launcher.setPosition(1);
            }
            if(gamepad2.y){
                robot.launcher.setPosition(0);
            }


            telemetry.addData("servo", robot.launcher.getPosition());
            telemetry.update();
        }
    }
}
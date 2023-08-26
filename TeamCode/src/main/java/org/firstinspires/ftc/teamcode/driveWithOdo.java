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

        waitForStart();

        while (opModeIsActive()) {

            //robot.mecanumDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 1);

            //telemetry.addData("motorRFPower", robot.motorRF.getPower());
            //telemetry.addData("motorRBPower", robot.motorRB.getPower());
            //telemetry.addData("motorLBPower", robot.motorLB.getPower());
            //telemetry.addData("motorLFPower", robot.motorLF.getPower());

            robot.refresh(robot.odometers);

            telemetry.addData("left encoder", robot.leftEncoder);
            telemetry.addData("right encoder", robot.rightEncoder);
            telemetry.addData("perpendicular encoder", robot.perpendicularEncoder);

            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "2023 TeleOp", group = "TeleOp")
/**
 * Programmer:
 * Date Created:  7/30/2022
 * Purpose: This is going to be our main teleop for PowerPlay, but for now is just to test on the new base when it is finished.
 */
public class TeleOp2023 extends LinearOpMode
{
    OpMode opmode;

    @Override
    public void runOpMode() {
        Hardware h = new Hardware();

        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }

        telemetry.addData("Main Initialization ", "complete");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("motorFrontLeft: ", h.motorFrontLeft.getDirection());
            telemetry.addData("motorFrontRight: ", h.motorFrontRight.getDirection());
            telemetry.addData("motorBackLeft: ", h.motorBackLeft.getDirection());
            telemetry.addData("motorBackRight: ", h.motorBackRight.getDirection());
            telemetry.update();
            h.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            /**Start drive system**/
            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}

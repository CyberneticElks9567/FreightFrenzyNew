package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "NewSkystoneTeleop", group = "TeleOp")
@Disabled
public class TestingTeleOp extends LinearOpMode
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
        /*h.motorArm.setPower(1);
        h.motorArm.setTargetPosition(150);
        sleep(1000);
        h.servoStopper.setPosition(.8);
        sleep(1000);
        h.motorArm.setTargetPosition(0);
        telemetry.addData("Movement ", "complete");*/
        while (opModeIsActive())
        {

            telemetry.addData("motorWinch Position: ", h.motorWinch.getCurrentPosition());
            telemetry.addData("motorSwivel Position: ", h.motorSwivel.getCurrentPosition());
            telemetry.addData("motorArm Position: ", h.motorArm.getCurrentPosition());

            //telemetry.addData("targetSwivel Value:", swivelTarget);
            telemetry.addData("Distance: ",h.distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Alpha", h.colorSensor.alpha());
            telemetry.addData("Red  ", h.colorSensor.red());
            telemetry.addData("Green", h.colorSensor.green());
            telemetry.addData("Blue ", h.colorSensor.blue());
            telemetry.update();
            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

         //The motorSwivel code used for testing.

            if (gamepad1.left_bumper /*&& h.motorSwivel.getCurrentPosition() <*/)
            {
                h.motorSwivel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                h.motorSwivel.setPower(-0.3);
            }

            if (gamepad1.right_bumper /*&& h.motorSwivel.getCurrentPosition() <*/)
            {
                h.motorSwivel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                h.motorSwivel.setPower(0.3);
            }
            if(!gamepad1.right_bumper && !gamepad1.left_bumper) 
            {
                h.motorSwivel.setTargetPosition(h.motorSwivel.getCurrentPosition());
                h.motorSwivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            //The program used to grab the blocks.

            if (gamepad1.a)
            {
                h.servoSwivel.setPosition(1);
            }
            if(gamepad1.b)
            {
                h.servoSwivel.setPosition(0);
            }

            //The Stopper code that is used to angle the Xrail.
            //
            /*if (gamepad1.x)
            {

                //position to go to
                h.servoStopper.setPosition(-0.5);


            }*/
            if (gamepad1.y)
            {
                h.servoStopper.setPosition(0);
            }
            if(gamepad1.x)
            {
                h.servoStopper.setPosition(.5);
            }

            //motorArm using the variables.


            //motorArm Limits:
            //Upper 280
            //Lower -100

            if (gamepad1.right_trigger == 1)
            {
                h.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                h.motorArm.setPower(1);
            }

            else if (gamepad1.left_trigger == 1)
            {
                h.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                h.motorArm.setPower(-0.15);
            }

            if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0)
            {
                h.motorArm.setPower(0);
            }

            /*else
            {
                h.motorArm.setTargetPosition(h.motorArm.getCurrentPosition());
                h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorArm.setPower(0.8);

            }*/
            //motorWinch is the extension code.
            /*h.motorWinch.setPower(1);
            if (gamepad1.dpad_left /*&& h.motorWinch.getCurrentPosition() < 600)
            {
                winchTarget++;
            }
            if (gamepad1.dpad_right /*&& h.motorWinch.getCurrentPosition() >  0)
            {
                winchTarget--;
            }
            if (!gamepad1.dpad_right && !gamepad1.dpad_left)
            {
                //h.motorWinch.setPower(0);
            }
            h.motorWinch.setTargetPosition(winchTarget);*/
            if(gamepad1.dpad_left /*&& h.motorWinch.getCurrentPosition() < 680*/)
            {
                h.motorWinch.setPower(1);
            }
            if(gamepad1.dpad_right /*&& h.motorWinch.getCurrentPosition() > 0*/)
            {
                h.motorWinch.setPower(-.5);
            }
            if (!gamepad1.dpad_left && !gamepad1.dpad_right)
            {
                h.motorWinch.setPower(0);
            }

            /*if (gamepad1.x)
            {
                h.servoSuction.setPosition(1);
            }
            if (gamepad1.y)
            {
                h.servoSuction.setPosition(-1);
            }*/


        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//TODO Add limts to other parts of robot, possibly to main TeleOp
@TeleOp(name = "Demo TeleOp - FOR PRESENTATION", group = "TeleOp")
/**
 * Programmer:    Sean Pakros & Kairon Johnson
 * Date Created:  5/28/22
 * Purpose: This is a TeleOp that we will use for demo and presentation purposes for others to try out.
 */

public class DemoMode extends LinearOpMode
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

        boolean pressedLastIterationIntake = false;
        boolean pressedLastIterationFast = false;
        boolean fastToggle = false;
        boolean pressedLastIterationWrist = false;
        boolean pressedLastIterationCarouselReverse = false;
        double armSpeedUp = -1;
        double armSpeedDown = .8;
        boolean limitSwitch = true;
        double wristPos = .5;
        boolean dpadDown = false, dpadUpPressed = false, dpadDownPressed = false;
        double winchPow = .5;

        waitForStart();
        while (opModeIsActive()) {
            boolean changed = false;
            boolean pressedIntake = gamepad1.x;
            boolean fast = gamepad2.right_stick_button;
            telemetry.addData("motorWinch Position: ", h.motorWinch.getCurrentPosition() + " busy =" + h.motorWinch.isBusy());
            telemetry.addData("motorWinch Power", winchPow);
            telemetry.addData("motorArm Position: ", h.motorArm.getCurrentPosition() + " busy =" + h.motorArm.isBusy());
            telemetry.addData("motorArm Power: ", h.motorArm.getPower());
            telemetry.addData("servoIntake: ", h.servoIntake.getPosition());
            telemetry.addData("fast boolean", fast);
            telemetry.addData("fastToggle boolean", fastToggle);
            telemetry.update();
            h.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            if(fast && !pressedLastIterationFast)
            {
                fastToggle = !fastToggle;
            }

            /**Start drive system**/
            if (fastToggle) {
                h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            } else {
                h.driveOmniDir(gamepad1.left_stick_x/2, gamepad1.left_stick_y/2, gamepad1.right_stick_x/2);
            }

            if(gamepad2.back) {
                requestOpModeStop();
            }

            /** These are what I call "Fine Tuning Controls" (FTC) which are used for slow accurate movements
             *  for actions such as placing the shipping element these would probably be the first thing I remove
             *  if I needed more controls
             **/
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                h.motorFrontLeft.setPower(-.2);
                h.motorFrontRight.setPower(.2);
                h.motorBackLeft.setPower(-.2);
                h.motorBackRight.setPower(.2);
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                h.motorFrontLeft.setPower(.2);
                h.motorFrontRight.setPower(-.2);
                h.motorBackLeft.setPower(.2);
                h.motorBackRight.setPower(-.2);
            }
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                h.motorFrontLeft.setPower(.2);
                h.motorFrontRight.setPower(.2);
                h.motorBackLeft.setPower(.2);
                h.motorBackRight.setPower(.2);
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                h.motorFrontLeft.setPower(-.2);
                h.motorFrontRight.setPower(-.2);
                h.motorBackLeft.setPower(-.2);
                h.motorBackRight.setPower(-.2);
            }
            /** END Fine Tuning Controls**/

            if (gamepad1.b) {
                h.servoWrist.setPosition(.5);
                wristPos = .5;
            }
            if (gamepad1.back) {
                h.servoWrist.setPosition(.9);
                wristPos = .9;
            }


            /** Toggle code for opening and closing the claw, if you press x it will alternate between being closed and opened enough for one block
             *  If you press y it will open fully we rarely open it fully as it adds risk that we may grab two blocks
             **/
            //0 is opened-.57 is partial open, 1 is closed

            if(pressedIntake & !pressedLastIterationIntake)
            {
                if(h.servoIntake.getPosition() > .8)
                {
                    h.servoIntake.setPosition(.57); //0 .1
                }
                else
                {
                    h.servoIntake.setPosition(1); //1 .3
                }

            }
            if (gamepad1.y)
            {
                h.servoIntake.setPosition(.45);
            }

            /** END CLAW CONTROL**/
            /** Simple controls for the carousel spin one way when 'b' is pressed another way when 'x' is pressed **/
            if (gamepad2.b)
            {
                h.motorCarousel.setPower(.35); //.4
            }
            if (gamepad2.x)
            {
                h.motorCarousel.setPower(-.35); //.4
            }
            if (!gamepad2.x && !gamepad2.b)
            {
                h.motorCarousel.setPower(0);
            }
            /** END CAROUSEL CONTROL **/
            /** Emergency switch to disable the limits on the arm in case we start the arm in the wrong position **/
            if(gamepad2.back)
            {
                limitSwitch = !limitSwitch;
            }

            /** Our arm controls, this rotates the arm so we can reach the different levels. If 'a' on gamepad1 is held while moving the arm
             * it will move at half speed for more precision. This is helpful for precision placing such as the team shipping element**/


            if(gamepad1.right_trigger > .01 && h.motorArm.getCurrentPosition() < 1470)
            {
                h.motorArm.setPower(armSpeedDown);
            }
            if (gamepad1.right_bumper && h.motorArm.getCurrentPosition() > -200)
            {
                h.motorArm.setPower(armSpeedUp);
            }

            if((!gamepad1.right_bumper && gamepad1.right_trigger == 0) || (h.motorArm.getCurrentPosition() > 1470 || h.motorArm.getCurrentPosition() < -200))
            {
                h.motorArm.setPower(0);
            }

            if(fastToggle)
            {
                armSpeedDown = .8;
                armSpeedUp = -1;
            }
            else
            {
                armSpeedDown = .5;
                armSpeedUp = -.7;
            }
            //TODO update winch limits

            if(gamepad1.left_trigger > .01 && h.motorWinch.getCurrentPosition() < 380)
            {
                h.motorWinch.setPower(winchPow);
            }
            if (gamepad1.left_bumper && h.motorWinch.getCurrentPosition() >= 10)
            {
                h.motorWinch.setPower(-winchPow);
            }
            if((!gamepad1.left_bumper && gamepad1.left_trigger == 0) || (h.motorWinch.getCurrentPosition() > 380 || h.motorWinch.getCurrentPosition() <= 10))
            {
                h.motorWinch.setPower(0);
            }

            h.servoWrist.setPosition(wristPos);
            pressedLastIterationIntake = pressedIntake;
            pressedLastIterationFast = fast;

        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "2022 TeleOp -CHOOSE THIS ONE-", group = "TeleOp")
/**
 * Programmer:    Sean Pakros
 * Date Created:  9/25/21
 * Purpose: This is our main teleop for driver controlled in freight frenzy season
 */
public class TeleOp2022 extends LinearOpMode
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
        boolean pressedLastIterationWrist = false;
        boolean pressedLastIterationCarouselReverse = false;
        boolean slow = false;
        double armSpeedUp = -1;
        double armSpeedDown = .8;
        boolean limitSwitch = true;
        double wristPos =.5;
        boolean dpadDown = false, dpadUpPressed = false, dpadDownPressed = false;

        waitForStart();
        while (opModeIsActive()) {
            boolean changed = false;
            boolean pressedIntake = gamepad1.x;
            //telemetry.addData("range", String.format("%.01f in", h.distanceSensor.getDistance(DistanceUnit.INCH)));
            //telemetry.addData("Distance: ",h.distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("motorWinch Position: ", h.motorWinch.getCurrentPosition() + " busy =" + h.motorWinch.isBusy());
            telemetry.addData("motorArm Position: ", h.motorArm.getCurrentPosition() + " busy =" + h.motorArm.isBusy());
            telemetry.addData("servoIntake: ", h.servoIntake.getPosition());
            telemetry.addData("servoWrist: ", h.servoWrist.getPosition());
            telemetry.addData("motorFrontLeft: ", h.motorFrontLeft.getDirection());
            telemetry.addData("motorFrontRight: ", h.motorFrontRight.getDirection());
            telemetry.addData("motorBackLeft: ", h.motorBackLeft.getDirection());
            telemetry.addData("motorBackRight: ", h.motorBackRight.getDirection());
            /*telemetry.addData("servoWrist: ", h.servoWrist.getPosition());
            telemetry.addData("motorFrontLeft: ", h.motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight: ", h.motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackLeft: ", h.motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight: ", h.motorBackRight.getCurrentPosition());*/
            telemetry.addData("wristPos: ", wristPos);
            telemetry.addData("servo toggle last iteration: ", pressedLastIterationIntake);
            telemetry.addData("servo toggle: ", pressedIntake);
            telemetry.addData("Motor reversed: ", "test");
            telemetry.update();
            h.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            slow = gamepad1.a;
            /**Start drive system**/
            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, slow, 2);

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

            /*if (gamepad1.dpad_down)
                if (!dpadDownPressed)
                {
                    if (wristPos < .8)
                    {
                        wristPos += .1;
                    }
                    dpadDownPressed = true;
                }
                else
                    dpadDownPressed = false;
            if (gamepad1.dpad_up)
                if (!dpadUpPressed)
                {
                    if (wristPos > .2) {
                        wristPos -= .1;
                    }
                    dpadUpPressed = true;
                }
                else
                    dpadUpPressed = false;
                */
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
            /*if (gamepad1.x && !changed) {
                if (h.servoIntake.getPosition() == 1) {
                    h.servoIntake.setPosition(.57);
                } else {
                    h.servoIntake.setPosition(1);
                }
                changed = true;
            } else if (!gamepad1.x){
                changed = false;
            }*/
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
                h.servoIntake.setPosition(0);
            }

            /** END CLAW CONTROL**/

            /*if(pressedSl & !pressedLastIterationCarousel)
            {
                if(h.motorCarousel.getPower() == 0 && gamepad2.b)
                {
                    h.motorCarousel.setPower(.3); //.3
                }
                else if(h.motorCarousel.getPower() == 0)
                {
                    h.motorCarousel.setPower(-.3); //-.3
                }
                else
                {
                    h.motorCarousel.setPower(0);
                }

            }*/
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
            /*if(pressedCarouselReverse & !pressedLastIterationCarouselReverse)
            {
                if(h.motorCarousel.getPower() == 0)
                {
                    h.motorCarousel.setPower(-.3);
                }
                else
                {
                    h.motorCarousel.setPower(0);
                }*/

            /** Emergency switch to disable the limits on the arm in case we start the arm in the wrong position **/
            if(gamepad2.back)
            {
                limitSwitch = !limitSwitch;
            }

            /** Our arm controls, this rotates the arm so we can reach the different levels. If 'a' on gamepad1 is held while moving the arm
             * it will move at half speed for more precision. This is helpful for precision placing such as the team shipping element**/
            /*if(limitSwitch)
            {
                if(gamepad1.right_trigger > .01 && h.motorArm.getCurrentPosition() < 1470)
                {
                    h.motorArm.setPower(armSpeedDown);
                }
                if (gamepad1.right_bumper && h.motorArm.getCurrentPosition() > -200)
                {
                    h.motorArm.setPower(armSpeedUp);
                }
            }*/


            if(gamepad1.right_trigger > .01)
            {
                h.motorArm.setPower(armSpeedDown);
            }
            if (gamepad1.right_bumper)
            {
                h.motorArm.setPower(armSpeedUp);
            }



            if(!gamepad1.right_bumper && gamepad1.right_trigger == 0)
            {
                h.motorArm.setPower(0);
            }

            if(slow)
            {
                armSpeedDown = .4;
                armSpeedUp = -.5;
            }
            else
            {
                armSpeedDown = .8;
                armSpeedUp = -1;
            }
            
            /*if(gamepad1.b)
            {
                h.motorArm.setTargetPosition(500);
                h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorArm.setPower(.3);
            }
            if(gamepad1.a)
            {
                h.motorArm.setTargetPosition(0);
                h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorArm.setPower(.3);
            }*/


            if(gamepad1.left_trigger > .01 && h.motorWinch.getCurrentPosition() < 450)
            {
                h.motorWinch.setPower(.5);
            }
            if (gamepad1.left_bumper && h.motorWinch.getCurrentPosition() >= -10)
            {
                h.motorWinch.setPower(-.5);
            }
            if(!gamepad1.left_bumper && gamepad1.left_trigger == 0)
            {
                h.motorWinch.setPower(0);
            }
           /* if(gamepad1.y)
            {
                h.motorLaunch.setPower(gamepad1.left_trigger);
            }
            if(gamepad1.x)
            {
                h.motorLaunch.setPower(0);
            }*/
            /*if(gamepad1.left_trigger == 1 ) // Trigger Launcher Control
            {
                h.motorLaunch.setPower(1);
                //++h.launchValue;
            }
            else if (gamepad1.left_trigger < .5)
            {
                h.motorLaunch.setPower(0);
            }*/
            /*if (gamepad1.y)
            {
                h.motorLaunch.setPower(1);
            }
            else
            {
                h.motorLaunch.setPower(0);
            }
            if(pressed & !pressedLastIteration)
            {
                if(h.motorLaunch.getPower() == 0)
                {
                    h.motorLaunch.setPower(1);
                }
                else
                {
                    h.motorLaunch.setPower(0);
                }

            }
            pressedLastIteration = pressed;*/


            /*if (gamepad1.y) //y button toggle Launcher Control
            {
                ++launchValue;
            }
            switch (launchValue)
            {
                case 1:
                    h.motorLaunch.setPower(1);
                    break;
                case 2:
                    h.motorLaunch.setPower(0);
                    launchValue = 0;
                    break;
            }*/
            /*if(gamepad1.a)
            {
                h.servoIntake.setPosition(.7);
            }
            if(gamepad1.b)
            {
                h.servoIntake.setPosition(1);
            }*/
            /*if (gamepad1.a) //Intake Control
            {
                ++intakeValue;
            }
            switch (intakeValue)
            {
                case 1:
                    h.motorIntake.setPower(1);
                    h.motorLaunch.setPower(-0.3);
                    break;
                case 2:
                    h.motorIntake.setPower(0);
                    h.motorLaunch.setPower(0);
                    intakeValue = 0;
                    break;
                default:
                    intakeValue = 0;
            }*/

            /*if (gamepad1.left_bumper) //High Position
            {
                h.motorArm.setTargetPosition(1);
                h.motorArm.setPower(.6);

            }
            if (gamepad1.right_bumper) //Low Position
            {
                h.motorArm.setTargetPosition(-1);
                h.motorArm.setPower(.6);
            }
            if (!h.motorArm.isBusy())
            {
                h.motorArm.setPower(0);
            }*/


            h.servoWrist.setPosition(wristPos);
            pressedLastIterationIntake = pressedIntake;
        }
    }
}

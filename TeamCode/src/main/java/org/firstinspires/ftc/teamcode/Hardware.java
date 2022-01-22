package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class Hardware extends LinearOpMode
{
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontLeft;
    public DcMotor motorCarousel;
    public DcMotor motorArm;     //
    public DcMotor motorWinch;   //
    public DcMotor motorLift;
    public DcMotor motorSpinner;
    public DcMotor motorSwivel;
    public DcMotor motorLaunch;
    public DcMotor motorIntake;
    public DcMotor motorWorm;

    //public DcMotor motorLeftLaunch;

    Servo markerDropServo;
    Servo bucketFlapServo;
    Servo servoStopper;
    Servo servoSwivel;
    Servo servoFoundation1;
    Servo servoFoundation2;
    Servo servoSuction;
    Servo servoIntake; //

    CRServo servoSpin;
    CRServo servoSpin2;

    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    //Rev2mDistanceSensor distanceSensor;

    ModernRoboticsI2cGyro MRgyro;
    ModernRoboticsI2cRangeSensor MRRange;

    BNO055IMU imu;
    Orientation angles;
    private double previousHeading = 0; //Outside of method
    private double integratedHeading = 0;

    //private HardwareMap aMap;
    DcMotor.RunMode initialMode = null;

    int driveTime;


    private Telemetry telemetry;


    @Override
    public void runOpMode()
    {
    }

    public void init(HardwareMap aMap, Telemetry inputTelemetry)
    {
        telemetry = inputTelemetry;
        //this.aMap = inputOpMode.hardwareMap;
        //this.telemetry = inputOpMode.telemetry;
        
        motorFrontRight = aMap.dcMotor.get("motorFrontRight");
        motorBackRight = aMap.dcMotor.get("motorBackRight");
        motorBackLeft = aMap.dcMotor.get("motorBackLeft");
        motorFrontLeft = aMap.dcMotor.get("motorFrontLeft");
        motorArm = aMap.dcMotor.get("motorArm");
        motorWinch = aMap.dcMotor.get("motorWinch");
        motorCarousel = aMap.dcMotor.get("motorCarousel");
        //motorSwivel =aMap.dcMotor.get("motorSwivel");

        //motorLeftLaunch = aMap.dcMotor.get("motorLeftLaunch");
       // motorLaunch = aMap.dcMotor.get("motorLaunch");
            //motorIntake = aMap.dcMotor.get("motorIntake");
        //motorWorm = aMap.dcMotor.get("motorWorm");
        /*//motorLift = aMap.dcMotor.get("motorLift");
        //motorSpinner = aMap.dcMotor.get("motorSpinner");

        //markerDropServo = aMap.servo.get("markerDropServo");
        //bucketFlapServo = aMap.servo.get("bucketFlapServo");
        servoSwivel = aMap.servo.get("servoSwivel");
        servoStopper = aMap.servo.get("servoStopper");
        servoFoundation1 = aMap.servo.get("servoFoundation1");
        servoFoundation2 = aMap.servo.get("servoFoundation2");
        servoSuction = aMap.servo.get("servoSuction");
        */
        servoIntake = aMap.servo.get("servoIntake");
        /*
        servoSpin = aMap.crservo.get("servoSpin");
        servoSpin2 = aMap.crservo.get("servoSpin2");
*/
        MRgyro = aMap.get(ModernRoboticsI2cGyro.class, "MRgyro");
        //MRRange = aMap.get(ModernRoboticsI2cRangeSensor.class, "MRRange");*/

        //colorSensor = aMap.get(ColorSensor.class, "REVcolor");
        //distanceSensor = aMap.get(DistanceSensor.class, "REVcolor");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");


        //motorLaunch.setDirection(DcMotorSimple.Direction.FORWARD);
            //motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        //motorWorm.setDirection(DcMotorSimple.Direction.FORWARD);
        //motorLeftLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorArm.setTargetPosition(0);
        /*motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorWinch.setTargetPosition(0);
        motorWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWinch.setDirection(DcMotorSimple.Direction.FORWARD);
        motorWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //motorSwivel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorSwivel.setDirection(DcMotorSimple.Direction.FORWARD);
        /*motorSwivel.setTargetPosition(0);
        motorSwivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSwivel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSwivel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        motorCarousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //motorSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //motorSwivel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);


        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
    }


    public void drive(boolean forward, int distanceInches, double power)
    {


        int distanceEncodeVal;

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        distanceEncodeVal = (int) Math.round((distanceInches/(4* Math.PI))*1120);
        driveTime = (distanceInches/10)*1000;



        if(forward)
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
        }

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        while((motorFrontLeft.getCurrentPosition() > distanceEncodeVal) && opModeIsActive())
        {
            telemetry.addData("Running", "...");
            telemetry.update();
        }
        */

        //telemetry.addData("Running", "...");
        //telemetry.update();

        if(forward)
        {

            while (motorFrontRight.getCurrentPosition() < distanceEncodeVal - 20 && !isStopRequested())
            {

            }
        }
        else
        {

            while (motorFrontRight.getCurrentPosition() > -distanceEncodeVal + 20 && !isStopRequested())
            {

            }

        }

        //telemetry.addData("Finished", ".");
        //telemetry.update();



        /*motorFrontLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);*/

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);


    }
    public void drivePureEncoder(boolean forward, int distanceEncodeVal, double power)
    {



        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveTime = distanceEncodeVal;




        if(forward)
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
        }

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        while((motorFrontLeft.getCurrentPosition() > distanceEncodeVal) && opModeIsActive())
        {
            telemetry.addData("Running", "...");
            telemetry.update();
        }
        */

        //telemetry.addData("Running", "...");
        //telemetry.update();

        if(forward)
        {

            while (motorFrontRight.getCurrentPosition() < distanceEncodeVal - 20 && !isStopRequested())
            {

            }
        }
        else
        {

            while (motorFrontRight.getCurrentPosition() > -distanceEncodeVal + 20 && !isStopRequested())
            {

            }

        }

        //telemetry.addData("Finished", ".");
        //telemetry.update();



        /*motorFrontLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);*/

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);


    }



    public void strafe(boolean left, int distanceInches,double power)
    {
        int distanceEncodeVal;

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        distanceEncodeVal = -(int) Math.round((distanceInches/(4* Math.PI))*1120);
        driveTime = (distanceInches/10)*1000;





        if(left)
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        while((motorFrontLeft.getCurrentPosition() > distanceEncodeVal) && opModeIsActive())
        {
            telemetry.addData("Running", "...");
            telemetry.update();
        }
        */
        if(left)
        {

            while (motorFrontRight.getCurrentPosition() < -distanceEncodeVal + 20 && !isStopRequested()/* && end > System.currentTimeMillis()*/)
            {

            }
        }
        else
        {

            while (motorFrontRight.getCurrentPosition() > distanceEncodeVal - 20 && !isStopRequested()/* && end > System.currentTimeMillis()*/)
            {

            }

        }



        /*motorFrontLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);*/

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }
    public void strafePureEncoder(boolean left, int distanceEncodeVal,double power, int driveTimeInSeconds)
    {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveTime = driveTimeInSeconds * 1000;
        long beginning = System.currentTimeMillis();
        long end = beginning + driveTime;




        if(left)
        {
            motorFrontLeft.setTargetPosition(-distanceEncodeVal);
            motorFrontRight.setTargetPosition(distanceEncodeVal);
            motorBackLeft.setTargetPosition(distanceEncodeVal);
            motorBackRight.setTargetPosition(-distanceEncodeVal);
        }
        else
        {
            motorFrontLeft.setTargetPosition(distanceEncodeVal);
            motorFrontRight.setTargetPosition(-distanceEncodeVal);
            motorBackLeft.setTargetPosition(-distanceEncodeVal);
            motorBackRight.setTargetPosition(distanceEncodeVal);
        }

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        while((motorFrontLeft.getCurrentPosition() > distanceEncodeVal) && opModeIsActive())
        {
            telemetry.addData("Running", "...");
            telemetry.update();
        }
        */
        if(left)
        {

            while (motorFrontRight.getCurrentPosition() < -distanceEncodeVal + 20 && !isStopRequested() && System.currentTimeMillis() != end)
            {

            }
        }
        else
        {

            while (motorFrontRight.getCurrentPosition() > distanceEncodeVal - 20 && !isStopRequested())
            {

            }

        }



        /*motorFrontLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorBackRight.setTargetPosition(0);*/

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    public void driveOmniDir(double joystickX, double joystickY, double rotation)
     {
         /**                   | Forward and|
          *                    | Backwards  | Strafing | Turning |  */
        motorFrontRight.setPower(-joystickY - joystickX - rotation);
        motorBackRight.setPower(-joystickY + joystickX - rotation);
        motorFrontLeft.setPower(-joystickY + joystickX + rotation);
        motorBackLeft.setPower(-joystickY - joystickX + rotation);

         /*motorFrontRight.setPower(joystickY + joystickX - rotation);
         motorBackRight.setPower(joystickY - joystickX - rotation);
         motorFrontLeft.setPower(-joystickY + joystickX - rotation);
         motorBackLeft.setPower(-joystickY - joystickX - rotation);*/
    }
    public void driveTank(double joystickL, double joystickR, double joystickX)
    {
        /**                   | Forward,    |           |
         *                    | Backwards,  |  Strafing |
                              | and Turning |           | */
        motorFrontRight.setPower(-joystickR - joystickX);
        motorBackRight.setPower(-joystickR + joystickX);
        motorFrontLeft.setPower(-joystickL + joystickX);
        motorBackLeft.setPower(-joystickL - joystickX);

         /*motorFrontRight.setPower(joystickY + joystickX - rotation);
         motorBackRight.setPower(joystickY - joystickX - rotation);
         motorFrontLeft.setPower(-joystickY + joystickX - rotation);
         motorBackLeft.setPower(-joystickY - joystickX - rotation);*/
    }
    /*driveTank(gamepad1.left_stick_Y, gamepad1.right_stick_Y, gamepad1.left_stick_X);*/

    public double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;

    }

    public void turn(int targetDegrees, double power, double correctionPower)
    {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



//Right is all positive
//Left is all negative
//Straight is left positive. Right negative.
        if(targetDegrees == 0)
        {
            //Tell robot to correct to straight forward

            if(MRgyro.getIntegratedZValue() > 0)
            {
                //If the MRgyro reads back left from zero

                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(-power);
                motorFrontRight.setPower(power);
                motorBackRight.setPower(power);

                while(MRgyro.getIntegratedZValue() > targetDegrees)
                {

                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                    telemetry.update();

                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }

                motorFrontLeft.setPower(correctionPower);
                motorBackLeft.setPower(correctionPower);
                motorFrontRight.setPower(-correctionPower);
                motorBackRight.setPower(-correctionPower);

                while(MRgyro.getIntegratedZValue() < targetDegrees)
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                    telemetry.update();
                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
            }
            if(MRgyro.getIntegratedZValue() < 0)
            {
                //If the MRgyro reads back right from zero

                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(-power);
                motorBackRight.setPower(-power);

                while(MRgyro.getIntegratedZValue() < targetDegrees)
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                    telemetry.update();

                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
                motorFrontLeft.setPower(-correctionPower);
                motorBackLeft.setPower(-correctionPower);
                motorFrontRight.setPower(correctionPower);
                motorBackRight.setPower(correctionPower);

                while(MRgyro.getIntegratedZValue() > targetDegrees)
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                    telemetry.update();
                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
            }
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
        }
////////////////////////////////////////////////////////////////////////////////////////////////////
        if(targetDegrees > 0)
        {
            //TURNING LEFT
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);

            while(MRgyro.getIntegratedZValue() < targetDegrees)
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                telemetry.update();

                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }

            motorFrontLeft.setPower(correctionPower);
            motorBackLeft.setPower(correctionPower);
            motorFrontRight.setPower(-correctionPower);
            motorBackRight.setPower(-correctionPower);

            while(MRgyro.getIntegratedZValue() > targetDegrees)
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                telemetry.update();
                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
        }
        else
        {

            //TURNING RIGHT
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);

            while(MRgyro.getIntegratedZValue() > targetDegrees)
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                telemetry.update();

                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
            motorFrontLeft.setPower(-correctionPower);
            motorBackLeft.setPower(-correctionPower);
            motorFrontRight.setPower(correctionPower);
            motorBackRight.setPower(correctionPower);

            while(MRgyro.getIntegratedZValue() < targetDegrees)
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", MRgyro.getIntegratedZValue());
                telemetry.update();
                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public void turnIMU(int targetDegrees, double power, double correctionPower)
    {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Right is up to +180
        //Left is up to -180

        //Right is all positive
        //Left is all negative
        //Straight is left positive. Right negative.
        if(targetDegrees == 0 && !isStopRequested())
        {
            //Tell robot to correct to straight forward

            if(getIntegratedHeading() > 0 && !isStopRequested())
            {
                //If the MRgyro reads back left from zero

                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(-power);
                motorFrontRight.setPower(power);
                motorBackRight.setPower(power);

                while(getIntegratedHeading() > targetDegrees && !isStopRequested())
                {

                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", getIntegratedHeading());
                    telemetry.update();

                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }

                motorFrontLeft.setPower(correctionPower);
                motorBackLeft.setPower(correctionPower);
                motorFrontRight.setPower(-correctionPower);
                motorBackRight.setPower(-correctionPower);

                while(getIntegratedHeading() < targetDegrees && !isStopRequested())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", getIntegratedHeading());
                    telemetry.update();
                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
            }
            if(getIntegratedHeading() < 0 && !isStopRequested())
            {
                //If the MRgyro reads back right from zero

                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(-power);
                motorBackRight.setPower(-power);

                while(getIntegratedHeading() < targetDegrees && !isStopRequested())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", getIntegratedHeading());
                    telemetry.update();

                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
                motorFrontLeft.setPower(-correctionPower);
                motorBackLeft.setPower(-correctionPower);
                motorFrontRight.setPower(correctionPower);
                motorBackRight.setPower(correctionPower);

                while(getIntegratedHeading() > targetDegrees && !isStopRequested())
                {
                    telemetry.addData("Target Value: ", targetDegrees);
                    telemetry.addData("Current Value: ", getIntegratedHeading());
                    telemetry.update();
                    try {
                        Thread.sleep(20);
                    }catch(Exception e){}
                    idle();
                }
            }
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
        }
////////////////////////////////////////////////////////////////////////////////////////////////////
        if(targetDegrees > 0 && !isStopRequested())
        {
            //TURNING LEFT
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);

            while(getIntegratedHeading() < targetDegrees && !isStopRequested())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", getIntegratedHeading());
                telemetry.update();

                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }

            motorFrontLeft.setPower(correctionPower);
            motorBackLeft.setPower(correctionPower);
            motorFrontRight.setPower(-correctionPower);
            motorBackRight.setPower(-correctionPower);

            while(getIntegratedHeading() > targetDegrees && !isStopRequested())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", getIntegratedHeading());
                telemetry.update();
                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
        }
        else
        {

            //TURNING RIGHT
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);

            while(getIntegratedHeading() > targetDegrees && !isStopRequested())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", getIntegratedHeading());
                telemetry.update();

                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
            motorFrontLeft.setPower(-correctionPower);
            motorBackLeft.setPower(-correctionPower);
            motorFrontRight.setPower(correctionPower);
            motorBackRight.setPower(correctionPower);

            while(getIntegratedHeading() < targetDegrees && !isStopRequested())
            {
                telemetry.addData("Target Value: ", targetDegrees);
                telemetry.addData("Current Value: ", getIntegratedHeading());
                telemetry.update();
                try {
                    Thread.sleep(20);
                }catch(Exception e){}
                idle();
            }
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public void setDrivePower(float motorPower)
    {
        motorFrontLeft.setPower(motorPower);
        motorBackLeft.setPower(motorPower);
        motorFrontRight.setPower(motorPower);
        motorBackRight.setPower(motorPower);
    }
    public void motorTest()
    {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setPower(0.5);
        motorBackLeft.setPower(0.5);
        motorFrontRight.setPower(0.5);
        motorBackRight.setPower(0.5);
    }
    public void motorTester(DcMotor inputMotor)
    {
        inputMotor.setPower(1);
        inputMotor.getPower();

    }
}

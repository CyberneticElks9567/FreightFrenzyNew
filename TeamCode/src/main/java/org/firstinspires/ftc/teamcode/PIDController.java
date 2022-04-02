package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDController {
    double Kp;
    double Ki;
    double Kd;

    double integralSum = 0;
    double lastError = 0;
    boolean angleWrap = false;

    ElapsedTime timer = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }
    public PIDController(double Kp, double Ki, double Kd, boolean angleWrap) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.angleWrap = angleWrap;
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double output(double target, double state) {
        double error;
        double derivative;

        if (angleWrap)
        {
            error = angleWrap(target - state);
        } else {
            error = target - state;
        }

        derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        timer.reset();
        lastError = error;

        return out;

    }

    public double angleWrap(double radians)
    {
        while (radians > Math.PI)
        {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI)
        {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}

package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class LiftPID {
    private double kp, ki, kd;
    private double totalError, lastError, lastTime;
    private ArrayList<Double> errors;
    private int target, max;
    private boolean isClose;
    private double I2CDeadzone = 30;
    private double deadzone = 3;

    public LiftPID(double kp, double ki, double kd, int target, int max) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        lastTime = 0;
        this.target = target;
        this.max = max;
        errors = new ArrayList<Double>();
    }

    public double getP(double error) {
        return kp * error;
    }

    public double getI(double error) {
        if (Math.abs(error) < 0.001) totalError = 0;
        return ki * totalError;
    }

    public double getD(double currentError) {
        return kd * (currentError - lastError);
    }

    public boolean checkCorrection(double error){

        if(Math.abs(errors.get(errors.size()-1)) < Math.abs(error))
            return true;
        else
            return false;

    }
    public void setP(double p){
        kp = p;
    }
    public void setI(double i){ ki = i;}

    public double getCorrection(double error) {
        if (Math.abs(error) < 0.001) {
            return 0;
        }

        totalError += error;

        double output = getP(error) + getI(error) + getD(error);

        lastError = error;
        errors.add(output);


        return output;
    }

    public double getCorrectionPosition(double position){
        /*
        if(Math.abs(position-target) < 10) return 0;
        else if(position - target > 200) return -1;
        else if (position - target > 100) return -0.75;
        else if (position - target > 0) return -0.5;
         */
        if(Math.abs(position-target) < I2CDeadzone) isClose = true;
        else isClose = false;

        if(Math.abs(position-target) < deadzone) return 0;

        if((position > target) && (target < 100)) setI(0.2);
        else setI(0.03);

        return getCorrection((target-position)/max);
    }

    public int getTarget(){
        return target;
    }

    public boolean isClose(){ return isClose;}

    public ArrayList<Double> aman(){
        return errors;
    }

    public void setTarget(int target){
        this.target = target;
    }
}
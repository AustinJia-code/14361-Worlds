package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.*;

public class PyramidalMotionProfile {

    Servo servo;
    int conversion; //speed: /270, torque: /360*5, axon: /355
    double minAngle, maxAngle, initAngle, tarAngle; //Degree
    double curTime, pastTime;
    double minSpeed, maxSpeed; // °/s
    RunMode mode;

    public PyramidalMotionProfile(Servo servo, int conversion, int initAngle, int tarAngle, double minSpeed, double maxSpeed, RunMode mode){
        this.servo = servo;
        this.conversion = conversion;
        this.initAngle = initAngle;
        this.tarAngle = tarAngle;
        this.minSpeed = minSpeed;
        this.maxSpeed = maxSpeed;
        this.mode = mode;
    }

    public void setTarget(int target){
        curTime = System.currentTimeMillis();
        pastTime = curTime;
        this.initAngle = toAngle(servo.getPosition());
        this.tarAngle = target;
    }

    public double update(){ //USE THIS
        switch(mode) {
            case AUTO:
                return toServo(tarAngle);
        }
        if(Math.abs(toAngle(servo.getPosition())-tarAngle) < 5) return toServo(tarAngle);   //if servo and target are within 5 deg, set servo to target
        return servo.getPosition()                                                          //servo
                + (deltaX(distanceFromMid() * (maxSpeed - minSpeed)))                 //+ distance from mid scaled 0-1 times speed range
                + (Math.signum(distanceFromMid()) * deltaX(minSpeed));                      //+ min speed
    }

    public double distanceFromMid(){ //init 0, tar 90
        double mid = (initAngle + tarAngle) / 2.0;
        return (mid - toAngle(servo.getPosition())) / mid;
    }

    public double toAngle(double servoPosition){
        return servoPosition * conversion;
    }

    public double toServo(double angle){
        return angle/conversion;
    }

    // speed unit: °/s
    public double deltaX(double speed){
        curTime = System.currentTimeMillis()/1000.0;
        double deltaT = curTime - pastTime;                   //change in time, seconds
        double deltaX = toServo(deltaT * speed);        //change in servo position (0-1)
        pastTime = curTime;
        return deltaX;
    }
}
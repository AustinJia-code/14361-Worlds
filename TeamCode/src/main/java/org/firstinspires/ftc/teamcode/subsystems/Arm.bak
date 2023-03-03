package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.commands.RunMode.*;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class Arm implements Subsystem {
    private Servo leftArm, rightArm;
    private double INTAKING = 5, GROUND = 5, LIFTED = 45, SCORING = 182, BACKWARDS = 295, LEVEL = 210, VERTICAL = 130;
    private double update = 0.05;
    private double profileStartTime;
    private double profileTargetPosition = 5, profileStartPosition = 5;
    private double profileTravelDistance;
    private double positionEstimate;
    Double goTo;
    TrapezoidalMotionProfile profile;
    private RunMode runMode;

    public Arm(HardwareMap hardwareMap){
        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");
        rightArm.setDirection(Servo.Direction.REVERSE);

        //Max accel: 100°/s Max vel: 500°/s
        profile = new TrapezoidalMotionProfile(50, 500);
        profileStartPosition = INTAKING;
        runMode = RunMode.TELE;
    }

    public void setAutoPositions(int pos){
        runMode = AUTO;
        SCORING = pos;
    }

    public void customAutoPosition(double position){ SCORING = position;}

    public void setPosition(State state){
        switch(state){
            case INTAKING:
                setArms(INTAKING);
                break;
            case GROUND:
                setArms(GROUND);
                break;
            case LOW:
                setArms(LEVEL);
                break;
            case BACKWARDS:
                setArms(BACKWARDS);
                break;
            case LIFTED:
                setArms(LIFTED);
                break;
            default:
                setArms(SCORING);
        }
    }

    public void slamThatJawn(){
        setArms(SCORING+30);
    }

    public void setArms(double target){
        switch(runMode) {
            case AUTO:
                leftArm.setPosition(toServoPosition(target));
                rightArm.setPosition(toServoPosition(target));
                break;
            case TELE:
                if(target != profileTargetPosition) {
                    profileStartPosition = positionEstimate;
                    profileTargetPosition = target;
                    profileTravelDistance = profileTargetPosition - profileStartPosition;
                    profileStartTime = System.currentTimeMillis();
                }
        }
    }

    public void updateArms(){
        goTo = profile.motion_profile(Math.abs(profileTravelDistance), (System.currentTimeMillis() - profileStartTime)/1000);
        goTo = toServoPosition(Math.signum(profileTravelDistance) * goTo.doubleValue() + profileStartPosition);
        if(goTo.isNaN()) goTo = toServoPosition(profileStartPosition);
        positionEstimate = toAngle(goTo);
        rightArm.setPosition(goTo);
        leftArm.setPosition(goTo);
    }

    public double toServoPosition(double angle){
        return (angle/355);
    }

    public double toAngle(double position){
        return position * 355;
    }

    public void setVertical(){ setArms(VERTICAL); }

    //public double toAxonPosition(double angle) { return (angle/(360*5))*0.85;} //5 turn?
    //public double toServoPosition(double angle){
    //        return angle/(300); //gobilda torque
    //    }

    public String armReport(){
        return goTo.toString() + " " + profileStartPosition + " " + profileTravelDistance + " " + (System.currentTimeMillis() - profileStartTime)/1000;
    }
}

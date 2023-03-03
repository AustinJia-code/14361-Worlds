package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.State;

public class Claw implements Subsystem {

    private Servo claw, wrist;
    private DistanceSensor poleLeftSensor, poleRightSensor, dropSensor, coneDistance;
    private int supinatedAngle = 35;
    private int pronatedAngle = 215;
    private int wristAngle = 35;
    private boolean pronated = false;
    private boolean flipped = false;
    double coneInches;
    boolean coneDetected, leftDetected, rightDetected, middleDropDetected, leftDropDetected, rightDropDetected, open;
    boolean autoDrop = false;

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.REVERSE);
        wrist = hardwareMap.servo.get("wrist");

        poleLeftSensor = hardwareMap.get(DistanceSensor.class, "pole right");
        poleRightSensor = hardwareMap.get(DistanceSensor.class, "pole left");
        dropSensor = hardwareMap.get(DistanceSensor.class, "drop sensor");
        coneDistance = hardwareMap.get(DistanceSensor.class, "cone sensor");
    }

    public void setPosition(State state){
        switch(state){
            case LIFTED:
            case INTAKING:
                open();
                pronate();
                break;
            case BACKWARDS:
                open();
                pronate();
                break;
            case GROUND:
                close();
                pronate();
                break;
            default:
                close();
                supinate();
        }
    }

    public void close(){
        claw.setPosition(toServoPosition(122));
        open = false;
    }

    public void TSEOpen(){
        claw.setPosition(toServoPosition(165));
        open = true;
    }

    public void open(){
        claw.setPosition(toServoPosition(182));
        open = true;
    }

    public void actuate(){
        if (!open) {
            open();
            open = true;
        } else {
            close();
            open = false;
        }
    }

    public void pronate() {
        wrist.setPosition(toServoPosition(pronatedAngle));
        wristAngle = pronatedAngle;
        pronated = true;
    }       //normal position

    public void supinate(){
        wrist.setPosition(toServoPosition(supinatedAngle));
        wristAngle = supinatedAngle;
        pronated = false;
    }

    public void outtake(){
        if(!flipped) {
            supinate();
        }else{
            pronate();
        }
    }   //scoring

    public void flip() {
        flipped = !flipped;
        if(pronated) supinate();
        else pronate();
    }

    public void setLeft(){
        wrist.setPosition(toServoPosition(wristAngle+27));
        pronated = false;
    }

    public void setRight(){
        wrist.setPosition(toServoPosition(wristAngle-27));
        pronated = false;
    }

    public void setAdjustment(boolean left, boolean middle, boolean right){
        double value = 0;
        int recognitions = 0;
        if(left){
            value -= 1;
            recognitions += 1;
        }if(middle){
            value += 0;
            recognitions += 1;
        }if(right){
            value += 1;
            recognitions += 1;
        }

        wrist.setPosition(toServoPosition((int)(wristAngle + (value / recognitions) * 20)));
        pronated = false;
    }

    public void intakeUpdate(){
        coneInches = coneDistance.getDistance(DistanceUnit.INCH);
        coneDetected = coneInches < 6.75;
        if(coneDetected){
            close();
        }
    }

    //!!!flip caching until coen not detected to flip??
    public boolean outtakeUpdate(State state, Gamepad gamepad1, Gamepad gamepad2, int loop){
        double[] sensors = {poleLeftSensor.getDistance(DistanceUnit.INCH), dropSensor.getDistance(DistanceUnit.INCH), poleRightSensor.getDistance(DistanceUnit.INCH)};
        if(loop % 5 != 0) return false;
        switch (state) {
            case HIGH:
            case MIDDLE:
            case LOW:
                leftDetected = sensors[0] < 10;
                rightDetected = sensors[2] < 10;
                middleDropDetected = sensors[1] <= 8.5 && sensors[1]>= 7;
                leftDropDetected = sensors[0] <= 8.5 && sensors[0] >= 7;
                rightDropDetected = sensors[2] <= 8.5 && sensors[2] >= 7;

                if ((leftDetected && rightDetected) || (!leftDetected && !rightDetected)){
                    outtake();
                } else if (rightDetected) {
                    setLeft();
                } else if (leftDetected) {
                    setRight();
                }

                if ((middleDropDetected || leftDropDetected || rightDropDetected) && !open) {
                    gamepad2.setLedColor(0, 1, 0, 100);
                    gamepad2.rumble(300);
                    gamepad1.setLedColor(0, 1, 0, 100);
                    gamepad1.rumble(300);
                    if(autoDrop) open();
                }else {
                    gamepad2.setLedColor(0,0.5,0.5, 100000);
                    gamepad1.setLedColor(0,0.5,0.5, 100000);
                }
                break;
            default:
                pronate();
                break;
        }
        return true;
    }

    public void setAutoDrop(){
        autoDrop = !autoDrop;
    }

    public double toServoPosition(double angle){
        return angle/270;
    }
    public double toAxonPosition(double angle) { return (angle/355);}

    public boolean getAutoDrop(){
        return autoDrop;
    }
}
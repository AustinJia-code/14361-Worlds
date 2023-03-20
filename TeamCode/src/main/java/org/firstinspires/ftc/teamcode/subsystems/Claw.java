package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.State;

public class Claw implements Subsystem {

    private Servo claw, wrist;
    private DistanceSensor behindLeftSensor, behindRightSensor, coneDistance, poleLeftSensor, poleRightSensor                       ;
    private int supinatedAngle = 35;
    private int pronatedAngle = 215;
    private int wristAngle = 35;
    private boolean pronated = false;
    private boolean flipped = false;
    private double[] sensors = {0, 0, 0};
    double coneInches;
    boolean coneDetected, leftDetected, rightDetected, middleDropDetected, leftDropDetected, rightDropDetected, open;
    boolean leftTilt, rightTilt;
    boolean autoDrop = false;
    private int noneCount = 0, rightCount = 0, leftCount = 0;

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.REVERSE);
        wrist = hardwareMap.servo.get("wrist");

        behindLeftSensor = hardwareMap.get(DistanceSensor.class, "behind right");
        behindRightSensor = hardwareMap.get(DistanceSensor.class, "behind left");

        poleLeftSensor = hardwareMap.get(DistanceSensor.class, "pole right");
        poleRightSensor = hardwareMap.get(DistanceSensor.class, "pole left");

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
        claw.setPosition(toServoPosition(125));
        open = false;
    }

    public void TSEOpen(){
        claw.setPosition(toServoPosition(153));
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
        supinate();
    }   //scoring

    public void flip() {
        flipped = !flipped;
        if(pronated) supinate();
        else pronate();
    }

    public void setLeft(){
        wrist.setPosition(toServoPosition(supinatedAngle+27));
        pronated = false;
    }

    public void setRight(){
        wrist.setPosition(toServoPosition(supinatedAngle-27));
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
    public boolean behindCheck(State state, int loop, Arm arm, LinearSlides slide){
        sensors = new double[]{behindLeftSensor.getDistance(DistanceUnit.INCH), 0, behindRightSensor.getDistance(DistanceUnit.INCH)};
        if(loop % 5 != 0) return false;
        switch (state) {
            case HIGH:
            case MIDDLE:
            case LOW:
                leftDetected = sensors[0] < 5;
                rightDetected = sensors[2] < 5;

                leftTilt = 4.5 < sensors[0] && sensors[0] < 10;
                rightTilt = 4.5 < sensors[2] && sensors[2] < 10;

                leftDropDetected = sensors[0] <= 4.2 && sensors[0] >= 2;
                rightDropDetected = sensors[2] <= 4.2 && sensors[2] >= 2;

                if ((leftDetected && rightDetected) || (!leftDetected && !rightDetected)){
                    outtake();

                    noneCount++; leftCount = 0; rightCount = 0;

                    if(noneCount > 2) arm.keep();
                    if(noneCount > 3) slide.keep();
                    return false;
                } else if (rightDetected) {
                    setLeft();

                    noneCount = 0; leftCount = 0; rightCount++;

                    if(rightCount > 2) arm.raise();
                    if(rightCount > 3) slide.lower();
                    return true;

                } else if (leftDetected) {
                    setRight();

                    noneCount = 0; leftCount++; rightCount = 0;

                    if(leftCount > 2) arm.raise();
                    if(leftCount > 3) slide.lower();
                    return true;
                }
                break;
            default:
                pronate();
                break;
        }
        return true;
    }

    public boolean outtakeUpdate(State state, int loop){
        sensors = new double[]{poleLeftSensor.getDistance(DistanceUnit.INCH), 0, poleRightSensor.getDistance(DistanceUnit.INCH)};
        if(loop % 5 != 0) return false;
        switch (state) {
            case HIGH:
            case MIDDLE:
            case LOW:
                leftDetected = 4.5 < sensors[0] && sensors[0] < 10;
                rightDetected = 4.5 < sensors[2] && sensors[2] < 10;

                if ((leftDetected && rightDetected) || (!leftDetected && !rightDetected)) outtake();
                else if (rightDetected) setLeft();
                else if (leftDetected) setRight();
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

    public double getLeft() { return leftCount; }
    public double getRight() { return rightCount; }
}
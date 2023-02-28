package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.commands.State;

public class Arm implements Subsystem {
    private Servo leftArm, rightArm;
    private double INTAKING = 5, GROUND = 5, SCORING = 182, BACKWARDS = 300, LEVEL = 210, VERTICAL = 130;
    private double update = 0.05;
    private double position;

    public Arm(HardwareMap hardwareMap){
        leftArm = hardwareMap.servo.get("leftArm");
        leftArm.setDirection(Servo.Direction.REVERSE);

        rightArm = hardwareMap.servo.get("rightArm");
    }

    public void setAutoPositions(int pos){
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
            default:
                setArms(SCORING);
        }
    }

    public void slamThatJawn(){
        setArms(SCORING+30);
    }

    public void setArms(double position){
        this.position = toServoPosition(position);
        leftArm.setPosition(toServoPosition(position));
        rightArm.setPosition(toServoPosition(position));
    }

    public double toServoPosition(double angle){
        return angle/(300);
    }

    public void setVertical(){ setArms(VERTICAL); }

    public double toAxonPosition(double angle) { return (angle/(360*5))*0.85;}
}

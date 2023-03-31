package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.commands.*;

public class Arm implements Subsystem {
    private Servo leftArm, rightArm;
    private double INTAKING = 5, GROUND = 5, LIFTED = 150, SCORING = 182, BACKWARDS = 291, LEVEL = 193, VERTICAL = 130;
    private double update = 0.05;

    Double goTo;
    TrapezoidalMotionProfile profile;
    State state;
    private RunMode runMode;

    public Arm(HardwareMap hardwareMap){
        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");
        rightArm.setDirection(Servo.Direction.REVERSE);
    }

    public void setAutoPositions(int pos){
        SCORING = pos;
    }

    public void customAutoPosition(double position){ SCORING = position;}

    public void setPosition(State state){
        switch(state){
            case INTAKING:
                this.state = State.INTAKING;
                setArms(INTAKING);
                break;
            case GROUND:
                this.state = State.GROUND;
                setArms(GROUND);
                break;
            case LOW:
                this.state = State.LOW;
                setArms(LEVEL);
                break;
            case BACKWARDS:
                this.state = State.BACKWARDS;
                setArms(BACKWARDS);
                break;
            case LIFTED:
                this.state = State.LIFTED;
                setArms(LIFTED);
                break;
            case MIDDLE:
                this.state = State.MIDDLE;
            case HIGH:
                this.state = State.HIGH;
            default:
                setArms(SCORING);
        }
    }

    public void setRaised(){
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
            case MIDDLE:
                setArms(SCORING - 17);
                break;
            case HIGH:
                setArms(SCORING - 23);
                break;
            default:
                setArms(SCORING);
        }
    }

    public void slamThatJawn(){
        setArms(SCORING+30);
    }

    public void keep(){ setArms(SCORING); }
    public void raise(){ setRaised(); }

    public void setArms(double target){
        leftArm.setPosition(toServoPosition(target));
        rightArm.setPosition(toServoPosition(target));
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

    /*
    public String armReport(){
        return goTo.toString() + " " + profileStartPosition + " " + profileTravelDistance + " " + (System.currentTimeMillis() - profileStartTime)/1000;
    }
    */

}

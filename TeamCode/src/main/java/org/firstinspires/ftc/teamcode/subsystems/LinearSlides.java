package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.commands.*;

import static org.firstinspires.ftc.teamcode.commands.State.*;

import ftc.rogue.blacksmith.listeners.*;

public class LinearSlides implements Subsystem {

    enum Mode {POSITION, POWER}

    private DcMotorEx leftSlide, rightSlide;

    private LiftPID leftPID, rightPID;
    private int HIGH = spoolChange(1390), MIDDLE = spoolChange(620), LOW = 0, INTAKE = -7;
    private int FIVE = spoolChange(410), FOUR = spoolChange(308), THREE = spoolChange(208), TWO = spoolChange(75), ONE = 00;
    public int offset = 0;
    public boolean lowered = false;
    int target;

    Mode mode = Mode.POWER;

    private int update;

    public LinearSlides(HardwareMap hardwareMap){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPID = new LiftPID(3, 1, 1, 0, HIGH);
        rightPID = new LiftPID(3, 1, 1, 0, HIGH);

        leftSlide.setPower(0);
        rightSlide.setPower(0);

        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);

        setPosition(INTAKING);

        update = 20;
    }

    public void setPosition(State state){
        switch(state){
            case LIFTED:
            case INTAKING:
                setTarget(INTAKE+offset);
                break;
            case BACKWARDS:
                setTarget(INTAKE+offset);
                break;
            case GROUND:
                setTarget(INTAKE+offset);
                break;
            case LOW:
                setTarget(LOW+offset);
                break;
            case MIDDLE:
                setTarget(MIDDLE+offset);
                break;
            case HIGH:
                setTarget(HIGH+offset);
                break;
            default:
                setTarget(INTAKE+offset);
                break;
        }
        lowered = false;
    }

    public void lower(){
        if(!lowered) {
            setTarget(target - spoolChange(140));
        }
        lowered = true;
    };
    public void keep(){
        if(lowered) {
            setTarget(target + spoolChange(140));
        }
        lowered = false;
    };

    public void midLilHigher(){
        setTarget(MIDDLE+100);
    }

    public void highLilHigher(){
        setTarget(HIGH+20);
    }

    public void incrementSlides(double input){
        switch(mode) {
            case POWER:
                if (Math.abs(input) > 0.01) {
                    setTarget(
                            Range.clip(rightPID.getTarget() + (int) Math.round(input * update), INTAKE + offset, spoolChange(1650) + offset)
                    );
                }
                break;
            case POSITION:
                setTarget((int)(rightSlide.getCurrentPosition()+input));
        }
    }

    public void setTarget(int position){
        target = position;
        switch(mode){
            case POWER:
                rightPID.setTarget(position);
                leftPID.setTarget(position);
            case POSITION:
                rightSlide.setTargetPosition(position);
                leftSlide.setTargetPosition(position);
        }
    }

    public void powerSlides(){
        double power = rightPID.getCorrectionPosition(rightSlide.getCurrentPosition());

        rightSlide.setPower(power);
        leftSlide.setPower(power);
    }

    public void setModeToPosition(){
        HIGH = spoolChange(1410); MIDDLE = spoolChange(675); LOW = spoolChange(95); INTAKE = spoolChange(95);
        mode = Mode.POSITION;
        rightSlide.setTargetPosition(0);
        leftSlide.setTargetPosition(0);

        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);

        rightSlide.setPower(1); //CHANGED FROM 1
        leftSlide.setPower(1); //CHANGED FROM 1
    }

    public void setModeToPosition(int lift){
        HIGH = spoolChange(1410 + lift); MIDDLE = spoolChange(675); LOW = spoolChange(95); INTAKE = spoolChange(95);
        mode = Mode.POSITION;
        rightSlide.setTargetPosition(0);
        leftSlide.setTargetPosition(0);

        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);

        rightSlide.setPower(1); //CHANGED FROM 1
        leftSlide.setPower(1); //CHANGED FROM 1
    }

    public void customScoringPositions(int HIGH, int MIDDLE){
        this.HIGH = HIGH;
        this.MIDDLE = MIDDLE;
    }

    public void customIntakingPositions(int FI, int FO, int TH, int TW, int ON){
        this.FIVE = FIVE + FI;
        this.FOUR = FOUR + FO;
        this.THREE = THREE + TH;
        this.TWO = TWO + TW;
        this.ONE = ONE + ON;
    }

    public void setFive(){
        setTarget(FIVE);
    }

    public void setFour(){
        setTarget(FOUR);
    }

    public void setThree(){
        setTarget(THREE);
    }

    public void setTwo(){
        setTarget(TWO);
    }

    public void setOne(){
        setTarget(ONE);
    }

    public String slideReport(){
        return(leftSlide.getCurrentPosition() + " " + rightSlide.getCurrentPosition() + " " + target + " " + offset);
    }

    public void reset(){
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static int spoolChange(int height){
        return (int) (height / 1.2 / 384.5 * 145.1);
    }
}
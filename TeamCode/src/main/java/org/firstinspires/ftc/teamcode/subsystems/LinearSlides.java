package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.*;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.commands.*;

import static org.firstinspires.ftc.teamcode.commands.State.*;

import ftc.rogue.blacksmith.listeners.*;

@Config
public class LinearSlides implements Subsystem {

    enum Mode {POSITION, POWER}

    private DcMotorEx leftSlide, rightSlide;

    private PIDController leftPID, rightPID;
    private int HIGH = spoolChange(1405), MIDDLE = spoolChange(680), LOW = 0, INTAKE = -10;
    private int FIVE = spoolChange(412), FOUR = spoolChange(293), THREE = spoolChange(190), TWO = spoolChange(72), ONE = 00;
    public int offset = 0;
    public boolean lowered = false;
    int target;

    Mode mode = Mode.POWER;

    private int update;

    public static double P = 0.032;
    public static double I = 0.000;
    public static double D = 0.00001;

    public LinearSlides(HardwareMap hardwareMap){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPID = new PIDController(P,I,D);
        rightPID = new PIDController(P,I,D);

        leftPID.setTolerance(10); rightPID.setTolerance(10);

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
                setTarget(INTAKE+offset + spoolChange(10));
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
        setTarget(HIGH+spoolChange(50));
    }

    public void incrementSlides(double input){
        switch(mode) {
            case POWER:
                if (Math.abs(input) > 0.01) {
                    setTarget(
                            Range.clip((int)leftPID.getSetPoint() + (int) Math.round(input * update), INTAKE + offset, spoolChange(1650) + offset)
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
                rightPID.setSetPoint(position);
                leftPID.setSetPoint(position);
            case POSITION:
                rightSlide.setTargetPosition(position);
                leftSlide.setTargetPosition(position);
        }
    }

    public void powerSlides(){
        double power = leftPID.calculate(leftSlide.getCurrentPosition());
            rightSlide.setPower(power);
            leftSlide.setPower(power);
        if(leftPID.atSetPoint()) { leftPID.reset(); }
    }

    public void setModeToPosition(){
        HIGH = spoolChange(1480); MIDDLE = spoolChange(675); LOW = spoolChange(95); INTAKE = spoolChange(95);
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
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setPower(0);
        rightSlide.setPower(0);

        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);

        setPosition(INTAKING);
    }

    public static int spoolChange(int height){
        return (int) (height / 1.2 / 384.5 * 145.1);
    }

    public int getError(){ return (int)leftPID.getPositionError(); }
    public int getTarget(){ return (int)leftPID.getSetPoint(); }
    public int getReal(){ return leftSlide.getCurrentPosition(); }
}
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.*;

import static org.firstinspires.ftc.teamcode.commands.State.*;

public class LinearSlides implements Subsystem {

    enum Mode {POSITION, POWER}

    private DcMotorEx leftSlide, rightSlide;

    private LiftPID leftPID, rightPID;
    private int HIGH = spoolChange(1400), MIDDLE = spoolChange(630), LOW = 0, INTAKE = 0;
    private int FIVE = spoolChange(430), FOUR = spoolChange(360), THREE = spoolChange(250), TWO = spoolChange(130), ONE = 00;
    public int offset = 0;
    public boolean lowered = false;
    int target;
    int stallCount;

    Mode mode = Mode.POWER;

    private int update, currentPosition;

    public LinearSlides(HardwareMap hardwareMap){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPID = new LiftPID(3, 0.17, 1, 0, 515);
        rightPID = new LiftPID(3, 0.17, 1, 0, 515);

        leftSlide.setPower(0);
        rightSlide.setPower( 0);

        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);

        setPosition(INTAKING);

        update = 20;
        stallCount = 0;
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
            case FIVE:
                setTarget(FIVE);
                break;
            case FOUR:
                setTarget(FOUR);
                break;
            case THREE:
                setTarget(THREE);
                break;
            case TWO:
                setTarget(TWO);
                break;
            case ONE:
                setTarget(ONE);
                break;
            default:
                setTarget(INTAKE+offset);
                break;
        }
        lowered = false;
    }

    public void lower(){
        if(!lowered) {
            setTarget(Math.max(0,target - spoolChange(250)));
        }
        lowered = true;
    };
    public void keep(){
        if(lowered) {
            setTarget(Math.max(0, target + spoolChange(250)));
        }
        lowered = false;
    };

    public void midLilHigher(){
        setTarget(MIDDLE+100);
    }

    public void highLilHigher(){
        setTarget(HIGH+20);
    }

    public void highLilLower(){
        setTarget(HIGH - 40);
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
                leftPID.clearError();
                rightPID.clearError();
                rightPID.setTarget(position);
                leftPID.setTarget(position);
            case POSITION:
                rightSlide.setTargetPosition(position);
                leftSlide.setTargetPosition(position);
        }
    }

    public void powerSlides(double voltage, State state, double override){
        int rightCurrent = rightSlide.getCurrentPosition();
        //currentPosition = Math.max(rightSlide.getCurrentPosition(), 0);
        double power = rightPID.getCorrectionPosition(rightCurrent, voltage, state);

        switch(state){
            case INTAKING:
            case BACKWARDS:
            case LIFTED:
            case LOW:
                if(Math.min(rightCurrent, leftSlide.getCurrentPosition()) < 5){
                    power = 0;
                    rightPID.setI(0);
                    rightPID.clearError();
                }
                break;
        }

        //power = stallCheck(power);
        if(override != 0){
            setTarget(rightCurrent);
            power = -override;
        }
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

    public String slideReport(){
        return(leftSlide.getCurrentPosition() + " " + rightSlide.getCurrentPosition() + " " + target + " " + offset);
    }

    public boolean isClose(){
        return rightPID.isClose();
    }

    public int getTarget(){
        return rightSlide.getTargetPosition();
    }
    public int getOffset(){return offset;}
    public double getLeftCurrent(){
        return leftSlide.getCurrent(CurrentUnit.AMPS);
    }

    public double getRightCurrent(){
        return rightSlide.getCurrent(CurrentUnit.AMPS);
    }

    public int getLeftPosition(){
        return leftSlide.getCurrentPosition();
    }

    public int getRightPosition(){
        return rightSlide.getCurrentPosition();
    }

    public double getPower(){
        return rightSlide.getPower();
    }

    public void reset(){
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setPower(0);
        rightSlide.setPower( 0);

        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public double stallCheck(double power){
        if(rightSlide.getCurrent(CurrentUnit.AMPS) > 5) {
            stallCount++;
        }else{
            stallCount = 0;
        }
        if(stallCount > 5){
            return 0;
        }
        return power;
    }

    public static int spoolChange(int height){
            return (int) (height / 1.2 / 384.5 * 145.1);
    }

    public double getTotalError(){
        return rightPID.getTotalError();
    }

    public double getCurrentPos(){
        return currentPosition;
    }
}
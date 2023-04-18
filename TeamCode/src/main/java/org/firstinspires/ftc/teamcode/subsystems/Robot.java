package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.commands.State.*;

import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.teamcode.commands.State;

public class Robot {

    public Mecanum drivetrain;
    public Arm arm;
    public Claw claw;
    public LinearSlides slide;

    private State state;
    Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        drivetrain = new Mecanum(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        slide = new LinearSlides(hardwareMap);
        state = INTAKING;
    }

    public void setPosition(State state){
        claw.setPosition(state);
        arm.setPosition(state);
        slide.setPosition(state);
        if(state.equals(FIVE)||state.equals(FOUR)||state.equals(THREE)||state.equals(TWO)||state.equals(ONE)) this.state = INTAKING;
        else this.state = state;
    }

    public void setState(State state){this.state = state;}
    public State getState(){
        return state;
    }
}

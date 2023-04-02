package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Mecanum implements Subsystem {
    public static SampleMecanumDrive drive;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    double y, x, rx, leftFrontPower, leftRearPower, rightFrontPower, rightRearPower, heading, rotX, rotY;
    private RevIMU imu;
    private Mode mode;
    private double speed = 0.85, desiredHeading = 0.0;

    enum Mode{FIELD, ROBOT}

    public Mecanum(HardwareMap hardwareMap) {
        drive = new SampleMecanumDrive(hardwareMap);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        imu = new RevIMU(hardwareMap);
        imu.init();
        mode = Mode.FIELD;
    }

    public void setMode(Mode m){
        mode = m;
    }
    public void recenter(){imu.reset();}
    public void switchModes(){
        if(mode.equals(Mode.ROBOT)){
            mode = Mode.FIELD;
        }else{
            mode = Mode.ROBOT;
        }
    }

    public String getMode(){
        if(mode == Mode.FIELD){
            return "FIELD CENTRIC";
        }else{
            return "ROBOT CENTRIC";
        }
    }

    public void drive(GamepadEx gamepad){
        y = Math.pow(gamepad.getLeftY(), 3);
        x = Math.pow(gamepad.getLeftX()*1.1, 3);
        rx = Math.pow(gamepad.getRightX(), 3);

        switch (mode){
            case FIELD:
                heading = Math.toRadians(-imu.getHeading()+180);
                x = x * Math.cos(heading) - y * Math.sin(heading);
                y = x * Math.sin(heading) + y * Math.cos(heading);
                break;
            case ROBOT:
                break;
        }
        leftFrontPower = (y + x + rx);
        leftRearPower = (y - x + rx);
        rightFrontPower = (y - x - rx);
        rightRearPower = (y + x - rx);

        powerMotors(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
    }

    private void powerMotors(double leftFrontPower, double leftRearPower, double rightFrontPower, double rightRearPower){
        leftFront.setPower(leftFrontPower * speed);
        leftRear.setPower(leftRearPower * speed);
        rightFront.setPower(rightFrontPower * speed);
        rightRear.setPower(rightRearPower * speed);
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }
}
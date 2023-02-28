package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.teamcode.commands.*;

public class DifferentialSwerve {
    private double maxV = 1150.0/60.0*145.1 * 0.8; //max rpm 80%
    private DcMotorEx TL, BL, TR, BR;
    private double x, y, rotation, TLv, TRv, BLv, BRv;
    private final int A = 1000, B = 1000;
    private BNO055IMU imu;
    private SwerveModule left, right;

    public DifferentialSwerve(HardwareMap hardwareMap) {
        initializeMotors(hardwareMap);

        left = new SwerveModule(145.1, 106.0/13 * 18.0/14, 1);
        right = new SwerveModule(145.1, 106.0/13 * 18.0/14, -1);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }
    public void drive(GamepadEx gamepad){
        x = Math.abs(gamepad.getLeftX()) > 0.01 ? gamepad.getLeftX() : 0; //sideways movement
        y = Math.abs(gamepad.getLeftY()) > 0.01 ? -gamepad.getLeftY() : 0; //forward movement
        rotation = Math.abs(gamepad.getRightX()) > 0.01 ? gamepad.getRightX() : 0; //rotational movement

        left.readEncoders(TL.getCurrentPosition(), BL.getCurrentPosition());
        right.readEncoders(TR.getCurrentPosition(), BR.getCurrentPosition());

        left.calc(y, x, rotation, -imu.getAngularOrientation().firstAngle);
        right.calc(y, x, rotation, -imu.getAngularOrientation().firstAngle);

        TLv = left.gearSpeed(-1, A, B);
        BLv = left.gearSpeed(1, A, B);
        TRv = right.gearSpeed(-1, A, B);
        BRv = right.gearSpeed(1, A, B);

        TL.setVelocity(TLv);
        BL.setVelocity(BLv);
        TR.setVelocity(TRv);
        BR.setVelocity(BRv);
    }

    private void initializeMotors(HardwareMap hardwareMap){
        TL = hardwareMap.get(DcMotorEx.class, "leftFront"); //1
        BL = hardwareMap.get(DcMotorEx.class, "leftRear"); //1
        TR = hardwareMap.get(DcMotorEx.class, "rightFront"); //1
        BR = hardwareMap.get(DcMotorEx.class, "rightRear"); //1

        TL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        TR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}

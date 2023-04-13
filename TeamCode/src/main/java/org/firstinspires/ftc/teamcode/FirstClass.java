package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@Disabled
@TeleOp(name="First Program")
public class FirstClass extends OpMode{
    private DcMotor Jazzy;
    @Override
    public void init() {
        Jazzy = hardwareMap.dcMotor.get("J");
        telemetry.addData("Status:", "Initialized");
    }

    @Override
    public void loop() {
        double test = gamepad1.left_trigger;
        Jazzy.setPower(test);
    }
}

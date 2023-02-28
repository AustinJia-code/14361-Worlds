package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class VoltageReader {

    private HardwareMap hardwareMap;
    double maxVoltage = 14;
    double minVoltage = 12;
    double minPower = 0.7;

    public VoltageReader(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public double multiplier(){
        return (hardwareMap.voltageSensor.iterator().next().getVoltage() - minVoltage)
                / (maxVoltage-minVoltage)
                * (1.0 - minPower)
                + minPower;
    }

    public double getVoltage(){
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }
}

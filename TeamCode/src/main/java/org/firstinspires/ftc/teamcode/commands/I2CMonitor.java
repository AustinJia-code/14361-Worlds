package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.*;

public class I2CMonitor {
    DistanceSensor sensor;
    int cutOff, defaultOutput;
    boolean on;
    double readValue;

    public I2CMonitor(DistanceSensor sensor, int cutOff, int defaultOutput){
        this.sensor = sensor;
        this.cutOff = cutOff;
        this.defaultOutput = defaultOutput;
    }

    public I2CMonitor(DistanceSensor sensor){
        this.sensor = sensor;
        this.cutOff = 30;
        this.defaultOutput = 15;
    }

    public double check(){
        readValue = defaultOutput;
        if(on){
            double startTime = System.currentTimeMillis();
            double temp = sensor.getDistance(DistanceUnit.INCH);
            if(System.currentTimeMillis()-startTime > cutOff){
                ;
            }else{
                on = false;
                readValue = temp;
            }
        }
        return readValue;
    }

    public double getDistance(DistanceUnit unit){
        return sensor.getDistance(unit);
    }

    public boolean working(){ return on; }

    public String toString(){ return sensor.toString(); };
}

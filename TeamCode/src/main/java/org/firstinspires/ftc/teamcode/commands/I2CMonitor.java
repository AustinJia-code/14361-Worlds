package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.*;

public class I2CMonitor {
    DistanceSensor sensor;
    int cutOff, defaultOutput, brokeCount;
    boolean on;
    double readValue;

    public I2CMonitor(DistanceSensor sensor, int cutOff, int defaultOutput){
        on = true;
        this.sensor = sensor;
        this.cutOff = cutOff;
        this.defaultOutput = defaultOutput;
        brokeCount = 0;
    }

    public I2CMonitor(DistanceSensor sensor){
        on = true;
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
                if(brokeCount++ > 5) ; on = false;
            }else{
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

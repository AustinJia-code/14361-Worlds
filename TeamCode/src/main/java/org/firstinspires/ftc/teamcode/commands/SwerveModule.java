package org.firstinspires.ftc.teamcode.commands;

public class SwerveModule {

    private double angle, rot, pivot, tick, travel, strafe, forward, speed;
    private int side;
    private boolean toInverse;
    private final double MOTOR_TICKS, RATIO;

    public SwerveModule(double MOTOR_TICKS, double RATIO, int side){ //left positive, right negative
        this.MOTOR_TICKS = MOTOR_TICKS;
        this.RATIO = RATIO;
        this.side = side;
    }
    public void readEncoders(double topTick, double bottomTick){
        tick = (topTick - bottomTick) / 2;
        rot = tick / MOTOR_TICKS / RATIO;
        angle = /*side * */Math.signum(rot)*(Math.abs(rot) - Math.floor(Math.abs(rot))) * 360.0;
    }
    public void calc(double y, double x, double rotation, double imu){
        strafe = Math.cos(Math.toRadians(imu))*x-Math.sin(Math.toRadians(imu))*y;
        forward = Math.sin(Math.toRadians(imu))*x+Math.cos(Math.toRadians(imu))*y - rotation*side;

        speed = Math.sqrt(Math.pow(strafe, 2) + Math.pow(forward, 2));
        setPivot(Math.toDegrees(Math.atan2(strafe, forward)));
    }

    public void setPivot(double target){
        pivot = Math.sin(Math.toRadians(angle - target));       //proportional controller
        travel = Math.abs((angle+360)%360-(target+360)%360);
        toInverse = Math.min(travel, 360.0 - travel) > 90.0;
        if(toInverse){
            speed *= -1;
            pivot *= -1;
        }
    }
    public double getAngle(){
        return angle;
    }
    public double gearSpeed(int hemisphere, int A, int B){
        return hemisphere * pivot * A - speed * B * side; //top negative, bottom positive
    }
}
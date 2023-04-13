package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.*;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.commands.State.*;

import org.firstinspires.ftc.teamcode.commands.VoltageReader;

import java.util.*;

@Config
public abstract class BasedAbstract extends OpMode {

    private Robot bot;

    private ElapsedTime runtime;
    private GamepadEx driver, operator;
    private VoltageReader voltageReader;
    int alliance;
    boolean canCheckI2C;
    int loop;
    double multiplier, loopTime;
    private boolean curRT, oldRT, curLT, oldLT, tilt, recess, locked;
    int section = 0;
    List<LynxModule> allHubs;
    ColorRangeSensor one;

    public abstract void setAlliance(); //-1 BLUE, 0 NEITHER, 1 RED

    public String allianceToString(){
        return alliance == -1 ? "BLUE" : (alliance == 0 ? "NEITHER" : "RED");
    }

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Status: Initializing");
        telemetry.update();

        setAlliance();

        bot = new Robot(hardwareMap, telemetry);
        loop = 0;

        voltageReader = new VoltageReader(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        tilt = true;
        recess = true;

        driver = new GamepadEx(gamepad1);   // drives the drivetrain
        operator = new GamepadEx(gamepad2); // controls the scoring systems
        runtime = new ElapsedTime();

        telemetry.addLine("Status: Initialized");
        telemetry.addLine("Alliance: " + allianceToString());
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        multiplier = 1;
        if(loop++ == 10000) loop = 0;

        double startTime = System.currentTimeMillis();

        // ---------------------------- LOOPING ---------------------------- //
        driver.readButtons();
        operator.readButtons();

        // ---------------------------- DRIVER CODE ---------------------------- //
        double desiredSpeed = 0;

        bot.drivetrain.drive(driver);

        if(driver.wasJustPressed(Button.Y)){                                        // DPad Up = Reset Gyro
            bot.drivetrain.recenter();
        }

        if(driver.getTrigger(Trigger.LEFT_TRIGGER) > 0.1){
            desiredSpeed = (0.8 - driver.getTrigger(Trigger.LEFT_TRIGGER) * 0.4) * multiplier;
        }else if(driver.getTrigger(Trigger.RIGHT_TRIGGER) > 0.1){
            desiredSpeed = 1 * multiplier;
        }else{
            desiredSpeed = 0.8 * multiplier;
        }

        if(bot.getState() == HIGH) desiredSpeed *= 0.95;

        bot.drivetrain.setSpeed(desiredSpeed);


        if(driver.getButton(Button.RIGHT_BUMPER)){
            bot.claw.open();
        }

        if(driver.wasJustPressed(Button.DPAD_UP)){
            bot.slide.offset += 10;
            bot.setPosition(bot.getState());
        }

        if(driver.wasJustPressed(Button.DPAD_DOWN)){
            bot.slide.offset -= 10;
            bot.setPosition(bot.getState());
        }

        if(driver.wasJustPressed(Button.DPAD_LEFT)){
            bot.slide.reset();
        }

        if(driver.wasJustPressed(Button.DPAD_RIGHT)){
            bot = new Robot(hardwareMap, telemetry);
        }

        if(driver.wasJustPressed(Button.A)){
            tilt = !tilt;
            recess = !recess;
        }

        if(driver.wasJustPressed(Button.X)){
            bot.claw.setLeft();
            bot.arm.raise();
        }

        if(driver.wasJustPressed(Button.B)){
            bot.claw.setRight();
            bot.arm.raise();
        }

        if(driver.wasJustPressed(Button.LEFT_BUMPER)){
            bot.drivetrain.switchModes();
        }

        // ---------------------------- OPERATOR CODE ---------------------------- //

        // ======================== OPERATOR LEFT TRIGGER LAYER ======================== //
        oldLT = curLT;
        if(operator.gamepad.left_trigger > 0.1){
            curLT = true;

            // CONESTACK
            if(operator.wasJustPressed(Button.DPAD_UP)){
                bot.setPosition(FIVE);
            }
            if(operator.wasJustPressed(Button.DPAD_RIGHT)){
                bot.setPosition(FOUR);
            }
            if(operator.wasJustPressed(Button.DPAD_LEFT)){
                bot.setPosition(THREE);
            }
            if(operator.wasJustPressed(Button.DPAD_DOWN)){
                bot.setPosition(TWO);
            }

            if(operator.wasJustReleased(Button.DPAD_UP)){
                bot.claw.close();
            }
            if(operator.wasJustReleased(Button.DPAD_RIGHT)){
                bot.claw.close();
            }
            if(operator.wasJustReleased(Button.DPAD_LEFT)){
                bot.claw.close();
            }
            if(operator.wasJustReleased(Button.DPAD_DOWN)){
                bot.claw.close();
            }

            if(operator.wasJustPressed(Button.A)){
                bot.slide.midLilHigher();
            }

        } else {

            // ======================== OPERATOR DEFAULT LAYER ======================== //

            curLT = false;

            if (operator.wasJustPressed(Button.RIGHT_BUMPER)) {                           // Right Bumper = Opens Claw, Goes to Floor
                if (bot.getState() == BACKWARDS) bot.setPosition(LIFTED);
                else bot.setPosition(INTAKING);
            }
            /*
            if (operator.isDown(Button.RIGHT_BUMPER)) {                           // Right Bumper = Opens Claw, Goes to Floor
                bot.claw.intakeUpdate();
            }
            */
            if (operator.wasJustReleased(Button.RIGHT_BUMPER)) {                            // Left Bumper = Closes Claw, Goes to Ground
                bot.claw.close();
            }
            if (operator.getTrigger(Trigger.RIGHT_TRIGGER) > 0.05) {
                if (!oldRT) {
                    bot.claw.setPosition(INTAKING);
                    bot.slide.setPosition(INTAKING);
                    bot.arm.setPosition(INTAKING);
                }
                oldRT = curRT;
                curRT = true;
            } else {
                oldRT = curRT;
                curRT = false;
                if (!curRT && oldRT) {
                    bot.claw.close();
                }
            }
            if (operator.wasJustPressed(Button.LEFT_BUMPER)) {                           // Right Bumper = Opens Claw, Goes to Floor
                if (bot.getState() == INTAKING) bot.setPosition(LIFTED);
                else if (bot.getState() == BACKWARDS || bot.getState() == LIFTED || bot.getState() == LOW)
                    bot.setPosition(BACKWARDS);
                else bot.setPosition(LOW);
            }
            if (operator.wasJustReleased(Button.LEFT_BUMPER)) {                            // Left Bumper = Closes Claw, Goes to Ground
                bot.claw.close();
            }

            if (operator.wasJustPressed(Button.DPAD_UP)) {                                      // Y = Sets High position
                bot.setPosition(HIGH);
            }

            if (operator.wasJustPressed(Button.DPAD_RIGHT)) {                                      // B = Sets Middle position
                bot.setPosition(MIDDLE);
            }

            if (operator.wasJustPressed(Button.DPAD_LEFT)) {                                      // A = Sets Low position
                bot.setPosition(LOW);
            }

            if (operator.wasJustPressed(Button.DPAD_DOWN)) {                                      // A = Sets Low position
                bot.setPosition(GROUND);
            }

            if (operator.wasJustPressed(Button.A)) {
                bot.claw.flip();
            }

            if (operator.wasJustPressed(Button.X)) {
                bot.claw.actuate();
            }

            if (operator.wasJustPressed(Button.Y)) {
                section = 0;
            }
            if (operator.wasJustPressed(Button.B)) {
                bot.claw.TSEOpen(++section % 4);
            }
            if (operator.wasJustReleased(Button.B)) {
                bot.claw.close();
            }
        }

        // CONESTACK RAISE ON TRIGGER RELEASE
        if(oldLT && !curLT){
            bot.slide.midLilHigher();
            oldLT = false;
        }

        if(bot.getState() != INTAKING && bot.getState() != BACKWARDS && bot.getState() != LOW) {
            //if(Math.abs(bot.slide.getPower()) < 0.03 ) {
            if(bot.slide.isClose()){
                canCheckI2C = true;
                if (recess) locked = bot.claw.behindCheck(bot.getState(), loop, bot.arm, bot.slide);
                if (tilt && !locked) bot.claw.outtakeUpdate(bot.getState(), loop);
            }else{
                canCheckI2C = false;
            }
        }
        bot.slide.powerSlides();
        bot.slide.incrementSlides(-operator.getRightY());            // Right Y = slowly raise the slides

        loopTime = System.currentTimeMillis() -startTime;
        // ---------------------------- TELEMETRY ---------------------------- //
        //teleTelemetry();
        //hubPowerTelemetry();
        //miscTelemetry();
        slideTelemetry();
    }

    @Override
    public void stop() {
        bot.setPosition(INTAKING);
        super.stop();
    }

    public void teleTelemetry(){
        telemetry.addLine("Runtime: " + runtime.toString());
        telemetry.addLine("Looptime: " + loopTime);
        telemetry.addLine("Multiplier: " + multiplier);
        telemetry.addLine("I2Cs: " + (tilt && canCheckI2C));
        telemetry.addLine("TSE: " + section%4);
        telemetry.addData("Mode: ", bot.drivetrain.getMode());
    }

    public void hubPowerTelemetry(){
        telemetry.addData("Voltage", voltageReader.getVoltage());
        for(int i = 0; i < allHubs.size(); i++){
            telemetry.addData("Current - Hub" + i, allHubs.get(i).getCurrent(CurrentUnit.AMPS));
        }
    }
    public void slideTelemetry(){
        telemetry.addData("Target: ", bot.slide.getTarget());
        telemetry.addData("Power: ", bot.slide.getPower());

        //telemetry.addData("LCurrent: ", bot.slide.getLeftCurrent());
        //telemetry.addData("RCurrent: ", bot.slide.getRightCurrent());

        //telemetry.addData("LPosition: ", bot.slide.getRightPosition());
        telemetry.addData("RPosition: ", bot.slide.getLeftPosition());

        telemetry.addData("Looptime: ", loopTime);

        //telemetry.addLine("Tilt: " + (tilt && canCheckI2C));

        //telemetry.addLine("Behind Right: " + bot.claw.getReadout());
    }

    public void miscTelemetry(){
        //telemetry.addData("Red: ", one.red());
        //telemetry.addData("Blue: ", one.blue());
        for(String s : bot.claw.brokenSensors()){
            telemetry.addData("Broken: ", s);
        }
    }
}

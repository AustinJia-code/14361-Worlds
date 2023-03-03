package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    int loop;
    double multiplier;
    private boolean curRT, oldRT;
    //List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

    public abstract void setAlliance(); //-1 BLUE, 0 NEITHER, 1 RED

    public String allianceToString(){
        return alliance == -1 ? "BLUE" : (alliance == 0 ? "NEITHER" : "RED");
    }

    @Override
    public void init() {
        telemetry.addLine("Status: Initializing");
        telemetry.update();

        telemetry.update();

        setAlliance();

        bot = new Robot(hardwareMap, telemetry);
        loop = 0;

        voltageReader = new VoltageReader(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        driver = new GamepadEx(gamepad1);   // drives the drivetrain
        operator = new GamepadEx(gamepad2); // controls the scoring systems
        runtime = new ElapsedTime();

        telemetry.addLine("Status: Initialized");
        telemetry.addData("Alliance:", allianceToString());
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        multiplier = 1;

        if(loop++ == 100000) loop = 0;

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
            desiredSpeed = (0.8 - driver.getTrigger(Trigger.LEFT_TRIGGER) * 0.6) * multiplier;
        }else if(driver.getTrigger(Trigger.RIGHT_TRIGGER) > 0.1){
            desiredSpeed = 1 * multiplier;
        }else{
            desiredSpeed = 0.8 * multiplier;
        }

        if(bot.getState() == HIGH) desiredSpeed *= 0.95;

        bot.drivetrain.setSpeed(desiredSpeed);

        /*
        if(driver.wasJustPressed(Button.LEFT_BUMPER)){
            bot.claw.setAutoDrop();
        }
        */

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

        if(driver.wasJustPressed(Button.LEFT_BUMPER)){
            bot.claw.TSEOpen();
        }

        // ---------------------------- OPERATOR CODE ---------------------------- //
        bot.slide.powerSlides();
        bot.arm.updateArms();

        if(operator.wasJustPressed(Button.RIGHT_BUMPER)){                           // Right Bumper = Opens Claw, Goes to Floor
            bot.setPosition(INTAKING);
        }
        if(operator.isDown(Button.RIGHT_BUMPER)){                           // Right Bumper = Opens Claw, Goes to Floor
            bot.claw.intakeUpdate();
        }
        if(operator.wasJustReleased(Button.RIGHT_BUMPER)){                            // Left Bumper = Closes Claw, Goes to Ground
            bot.claw.close();
        }
        if(operator.getTrigger(Trigger.RIGHT_TRIGGER) > 0.05){
            if(!oldRT){
                bot.claw.setPosition(INTAKING);
                bot.slide.setPosition(INTAKING);
                bot.arm.setPosition(INTAKING);
            }
            oldRT = curRT;
            curRT = true;
        }else{
            oldRT = curRT;
            curRT = false;
            if(!curRT && oldRT){
                bot.claw.close();
            }
        }
        if(operator.wasJustPressed(Button.LEFT_BUMPER)){                           // Right Bumper = Opens Claw, Goes to Floor
            if(bot.getState() == INTAKING) bot.setPosition(LIFTED);
            if(bot.getState() == LIFTED) bot.setPosition(BACKWARDS);
        }
        if(operator.wasJustReleased(Button.LEFT_BUMPER)){                            // Left Bumper = Closes Claw, Goes to Ground
            bot.claw.close();
        }

        if(operator.wasJustPressed(Button.DPAD_UP)){                                      // Y = Sets High position
            bot.setPosition(HIGH);
        }

        if(operator.wasJustPressed(Button.DPAD_RIGHT)){                                      // B = Sets Middle position
            bot.setPosition(MIDDLE);
        }

        if(operator.wasJustPressed(Button.DPAD_LEFT)){                                      // A = Sets Low position
            bot.setPosition(LOW);
        }

        if(operator.wasJustPressed(Button.DPAD_DOWN)){                                      // A = Sets Low position
            bot.setPosition(GROUND);
        }

        if(operator.wasJustPressed(Button.A)){
            bot.claw.flip();
        }

        if(operator.wasJustPressed(Button.X)){
            bot.claw.actuate();
        }

        if(operator.wasJustPressed(Button.B)){
            bot.claw.TSEOpen();
        }
        if(operator.wasJustReleased(Button.B)){
            bot.claw.close();
        }

        if(operator.wasJustReleased(Button.B)){
            bot.arm.setPosition(INTAKING);
        }

        if(operator.gamepad.touchpad_finger_1_x < 0 && operator.gamepad.touchpad_finger_1_y > 0 && operator.gamepad.touchpad_finger_1){
            bot.setPosition(INTAKING);
            bot.claw.open();
            bot.slide.setFive();
        }

        if(operator.gamepad.touchpad_finger_1_x > 0 && operator.gamepad.touchpad_finger_1_y > 0 && operator.gamepad.touchpad_finger_1){
            bot.setPosition(INTAKING);
            bot.claw.open();
            bot.slide.setFour();
        }

        if(operator.gamepad.touchpad_finger_1_x < 0 && operator.gamepad.touchpad_finger_1_y < 0 && operator.gamepad.touchpad_finger_1){
            bot.setPosition(INTAKING);
            bot.claw.open();
            bot.slide.setThree();
        }

        if(operator.gamepad.touchpad_finger_1_x > 0 && operator.gamepad.touchpad_finger_1_y < 0 && operator.gamepad.touchpad_finger_1){
            bot.setPosition(INTAKING);
            bot.claw.open();
            bot.slide.setTwo();
        }

        bot.claw.outtakeUpdate(bot.getState(), driver.gamepad, operator.gamepad, loop);
        bot.slide.incrementSlides(-operator.getRightY());            // Right Y = slowly raise the slides

        double loopTime = System.currentTimeMillis() -startTime;
        // ---------------------------- TELEMETRY ---------------------------- //
        telemetry.addData("Runtime:", runtime.toString());
        telemetry.addData("Looptime: ", loopTime);
        telemetry.addData("Multiplier: ", multiplier);
        telemetry.addData("Voltage: ", voltageReader.getVoltage());
        //telemetry.addData("Arm Report: ", bot.arm.armReport());
    }

    @Override
    public void stop() {
        bot.setPosition(INTAKING);
        super.stop();
    }
}

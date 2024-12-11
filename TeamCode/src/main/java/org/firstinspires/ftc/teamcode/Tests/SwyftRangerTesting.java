package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;;
@TeleOp(name = "Swyft Ranger Testing", group = "Sensor")

public class SwyftRangerTesting extends LinearOpMode {

    AnalogInput ranger1;
    AnalogInput ranger0;

    @Override
    public void runOpMode() {

        ranger1 = hardwareMap.get(AnalogInput.class, "distance_sensor_rear_3");
        ranger0 = hardwareMap.get(AnalogInput.class, "distance_sensor_rear_2");


        // get a reference to our Light Sensor object.
        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            //telemetry.addData("Raw Voltage 1",    ranger.getVoltage());
            //telemetry.addData("Inch 15DEG 0-1 Mode: ", (ranger.getVoltage()*32.5)-2.6);
            //telemetry.addData("Inch 20DEG 0-0 Mode 1: ", (ranger.getVoltage()*48.7)-4.9);
            //telemetry.addData("Inch 27DEG 1-0 Mode: ", (ranger.getVoltage()*78.1)-10.2);



            telemetry.addData("Raw Voltage 0",    ranger0.getVoltage());
            telemetry.addData("Inch 20DEG 0-0 Mode 0: ", (ranger0.getVoltage()*48.7)-4.9);
            telemetry.addData("Raw Voltage 1",    ranger1.getVoltage());
            telemetry.addData("Inch 20DEG 0-0 Mode 1: ", (ranger1.getVoltage()*48.7)-4.9);

            telemetry.update();
        }
    }
}
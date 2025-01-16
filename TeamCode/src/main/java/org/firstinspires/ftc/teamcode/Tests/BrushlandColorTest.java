package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class BrushlandColorTest extends LinearOpMode {

    RevColorSensorV3 colorSensi;
    Servo lightOne;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        colorSensi = hardwareMap.get(RevColorSensorV3.class, "intakeColorSensor");
        lightOne = hardwareMap.get(Servo.class, "lightOne");
        String chosenColor = "";


        waitForStart();
        while (opModeIsActive()) {
            NormalizedRGBA colors = colorSensi.getNormalizedColors();
            if (colors.red > 0.07 && colors.green > 0.07){
                chosenColor = "yellow";
            } else if (colors.red > 0.07){
                chosenColor = "red";
            } else if (colors.blue > 0.05){
                chosenColor = "blue";
            } else {
                chosenColor = "none";
            }

            lightOne.setPosition(getColorPWN(chosenColor));

            telemetry.addData("rbg: ", colors.red + " " + colors.blue + " " + colors.green);
            telemetry.addData("Color", chosenColor);
            telemetry.update();
        }

    }
    public double getColorPWN(String color){
       if (color.equals("red")){
           return 0.279;
       } else if (color.equals("blue")){
           return 0.611;
       } else if (color.equals("yellow")){
           return 0.388;
       } else {
           return 0;
       }
    }
}

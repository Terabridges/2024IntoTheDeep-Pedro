package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;

public class VisionSystem implements Subsystem {

    //Hardware
    public RevColorSensorV3 intakeColorSensor;
    public DistanceSensor leftBackDistance;
    public DistanceSensor rightBackDistance;
    public Camera frontCamera;
    public Camera backCamera;
    public TouchSensor magLimitSwitch;
    public Servo lightOne;

    //Software
    NormalizedRGBA colors;

    //Constructor
    public VisionSystem(HardwareMap map) {
        intakeColorSensor = map.get(RevColorSensorV3.class, "intake_color_sensor");
        leftBackDistance = map.get(DistanceSensor.class, "left_back_distance");
        rightBackDistance = map.get(DistanceSensor.class, "right_back_distance");
        frontCamera = map.get(Camera.class, "front_camera");
        backCamera = map.get(Camera.class, "back_camera");
        magLimitSwitch = map.get(TouchSensor.class, "limit_switch");
        lightOne = map.get(Servo.class, "light_one");
    }

    //Methods
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

    public String getColorVal(){
        if (colors.red > 0.07 && colors.green > 0.07){
            return "yellow";
        } else if (colors.red > 0.07){
            return "red";
        } else if (colors.blue > 0.05){
            return "blue";
        } else {
            return "none";
        }
    }

    public void detectColor(){
        colors = intakeColorSensor.getNormalizedColors();
    }

    public void setLightColor(String chosenColor) {
        lightOne.setPosition(getColorPWN(chosenColor));
    }

    //Interface Methods
    @Override
    public void toInit(){

    }

    @Override
    public void update(){
        detectColor();
        setLightColor(getColorVal());
    }
}

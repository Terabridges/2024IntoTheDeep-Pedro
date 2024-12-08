package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IntakeSystem implements Subsystem {

    //Hardware
    public static Servo leftLinear;
    public static Servo rightLinear;
    public static CRServo rightArm;
    public static CRServo leftArm;
    public static CRServo intake;
    public static AnalogInput leftArmAnalog;
    public static AnalogInput rightArmAnalog;

    //Software
    public static double RIGHT_SLIDE_RETRACT = 0.15;
    public static double RIGHT_SLIDE_EXTEND = 0;
    public static double LEFT_SLIDE_RETRACT = 0.85;
    public static double LEFT_SLIDE_EXTEND = 1;
    public static double INTAKE_PULL_IN = -0.4;
    public static double INTAKE_PULL_OUT = 0.4;

    //Constructor
    public IntakeSystem(HardwareMap map) {
        leftLinear = map.get(Servo.class, "left_linear");
        rightLinear = map.get(Servo.class, "right_linear");
        leftArm = map.get(CRServo.class, "leftArm");
        rightArm = map.get(CRServo.class, "right_arm");
        intake = map.get(CRServo.class, "intake");
        leftArmAnalog = map.get(AnalogInput.class, "left_analog");
        rightArmAnalog = map.get(AnalogInput.class, "right_analog");
    }

    //Methods
    public void setLinear(double lPos, double rPos) {
        leftLinear.setPosition(lPos);
        rightLinear.setPosition(rPos);
    }

    public void setArm(double lPow, double rPow) {
        leftArm.setPower(lPow);
        rightArm.setPower(rPow);
    }

    public void setIntake(double pow) {
        intake.setPower(pow);
    }

    //Interface Methods
    @Override
    public void toInit(){
        //startupcode
    }

    @Override
    public void update(){
    }

}

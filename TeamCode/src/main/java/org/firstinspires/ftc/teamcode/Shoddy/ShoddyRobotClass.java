package org.firstinspires.ftc.teamcode.Shoddy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class ShoddyRobotClass {
    private LinearOpMode myOpMode = null;

    //Wheel Motors
    public DcMotor LF;
    public DcMotor RF;
    public DcMotor LB;
    public DcMotor RB;

    //Other Motors
    public DcMotor topVertical;
    public DcMotor bottomVertical;

    //Servos
    public Servo leftLinear;
    public Servo rightLinear;
    public CRServo rightArm;
    public CRServo leftArm;
    public CRServo intake;

    //Analog
    public AnalogInput leftArmAnalog;
    public AnalogInput rightArmAnalog;

    //Other Variables

    //Drive Robot Variables
    public static double speedVar = 1;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public ShoddyRobotClass(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    //Methods
    public void wheelSetUp(){

        LF = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
        LB = myOpMode.hardwareMap.get(DcMotor.class, "left_back");
        RF = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
        RB = myOpMode.hardwareMap.get(DcMotor.class, "right_back");

        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);

    }
    public void servoSetUp(){
        rightLinear = myOpMode.hardwareMap.get(Servo.class, "right_linear");
        leftLinear = myOpMode.hardwareMap.get(Servo.class, "left_linear");
        rightArm = myOpMode.hardwareMap.get(CRServo.class, "right_arm");
        leftArm = myOpMode.hardwareMap.get(CRServo.class, "left_arm");
        intake = myOpMode.hardwareMap.get(CRServo.class, "intake");
    }
    public void motorSetUp(){
        topVertical = myOpMode.hardwareMap.get(DcMotor.class, "top_vertical");
        bottomVertical = myOpMode.hardwareMap.get(DcMotor.class, "bottom_vertical");
    }

    public void analogSetUp(){
        leftArmAnalog = myOpMode.hardwareMap.analogInput.get("left_analog");
        rightArmAnalog = myOpMode.hardwareMap.analogInput.get("right_analog");
    }

    public void driveRobot(double lf, double rf, double lb, double rb){
        // Send calculated power to wheels
        LF.setPower(lf);
        RF.setPower(rf);
        LB.setPower(lb);
        RB.setPower(rb);
    }
}

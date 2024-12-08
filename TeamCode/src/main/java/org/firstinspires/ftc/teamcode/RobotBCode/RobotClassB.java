package org.firstinspires.ftc.teamcode.RobotBCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.;


public class RobotClassB {
    private LinearOpMode myOpMode = null;

    //Wheel Motors
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;

    //Other Motors

        //A
    public DcMotor rightVerticalSlide;
    public DcMotor leftVerticalSlide;

        //B

    public DcMotor leftViper;
    public DcMotor rightViper;
    public DcMotor armMotorB;

    //Servos

        //A
    public Servo leftSlideServo;
    public Servo rightSlideServo;
    public CRServo rightArmServo;
    public CRServo leftArmServo;
    public CRServo intakeServo;

        //B
    public Servo armSlide;
    public Servo clawX;
    public Servo clawY;
    public Servo clawPinch;

    //Other Variables


    //Drive Robot Variables
    public static double speedVar = 1;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotClassB(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    //Methods
    public void wheelSetUpA(){

        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    public void wheelSetUpB(){

        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    public void servoSetUpA(){

        rightSlideServo = myOpMode.hardwareMap.get(Servo.class, "right_slide");
        leftSlideServo = myOpMode.hardwareMap.get(Servo.class, "left_slide");
        rightArmServo = myOpMode.hardwareMap.get(CRServo.class, "right_arm");
        leftArmServo = myOpMode.hardwareMap.get(CRServo.class, "left_arm");
        intakeServo = myOpMode.hardwareMap.get(CRServo.class, "intake");
    }
    public void motorSetUpA(){
        rightVerticalSlide = myOpMode.hardwareMap.get(DcMotor.class, "right_vertical");
        leftVerticalSlide = myOpMode.hardwareMap.get(DcMotor.class, "left_vertical");
    }

    public void servoSetUpB()
    {
        armSlide = myOpMode.hardwareMap.get(Servo.class, "armSlide");
        clawX = myOpMode.hardwareMap.get(Servo.class, "clawX");
        clawY = myOpMode.hardwareMap.get(Servo.class, "clawY");
        clawPinch = myOpMode.hardwareMap.get(Servo.class, "clawPinch");
    }

    public void motorSetUpB()
    {
        leftViper = myOpMode.hardwareMap.get(DcMotor.class, "leftViper");
        rightViper  = myOpMode.hardwareMap.get(DcMotor.class, "rightViper");
        armMotorB = myOpMode.hardwareMap.get(DcMotor.class, "armB");
    }

    public void driveRobot(double lf, double rf, double lb, double rb){
        // Send calculated power to wheels
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }

}

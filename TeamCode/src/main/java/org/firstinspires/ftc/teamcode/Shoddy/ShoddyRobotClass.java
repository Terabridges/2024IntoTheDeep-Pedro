package org.firstinspires.ftc.teamcode.Shoddy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;


public class ShoddyRobotClass {
    private OpMode myOpMode = null;

    //Wheel Motors
    public DcMotor LF;
    public DcMotor RF;
    public DcMotor LB;
    public DcMotor RB;

    //Other Motors
    public DcMotor topVertical;
    public DcMotor bottomVertical;
    //public DcMotor emptyMotor;

    //Servos
    public CRServo leftLinear;
    public CRServo rightLinear;
    public CRServo rightArm;
    public CRServo leftArm;
    public CRServo intake;
    public CRServo leftSwivel;
    public CRServo rightSwivel;
    public Servo wrist;
    public Servo claw;

    //Analog
    public AnalogInput leftArmAnalog;
    public AnalogInput rightSwivelAnalog;
    public AnalogInput rightLinearAnalog;

    public AbsoluteAnalogEncoder rightV4BEnc;
    public AbsoluteAnalogEncoder rightSwivelEnc;
    public AbsoluteAnalogEncoder rightLinearEnc;
    //Other Variables

    //Drive Robot Variables
    public static double speedVar = 1;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public ShoddyRobotClass(OpMode opmode) {
        myOpMode = opmode;
    }

    //Methods
    public void wheelSetUp(){

        LF = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
        LB = myOpMode.hardwareMap.get(DcMotor.class, "left_back");
        RF = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
        RB = myOpMode.hardwareMap.get(DcMotor.class, "right_back");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

    }
    public void servoSetUp(){
        rightLinear = myOpMode.hardwareMap.get(CRServo.class, "right_linear");
        leftLinear = myOpMode.hardwareMap.get(CRServo.class, "left_linear");

        rightArm = myOpMode.hardwareMap.get(CRServo.class, "right_arm");
        leftArm = myOpMode.hardwareMap.get(CRServo.class, "left_arm");

        intake = myOpMode.hardwareMap.get(CRServo.class, "intake");

        leftSwivel = myOpMode.hardwareMap.get(CRServo.class, "left_swivel");
        rightSwivel = myOpMode.hardwareMap.get(CRServo.class, "right_swivel");

        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");

        rightSwivel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLinear.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void motorSetUp(){
        topVertical = myOpMode.hardwareMap.get(DcMotor.class, "top_vertical");
        bottomVertical = myOpMode.hardwareMap.get(DcMotor.class, "bottom_vertical");

        topVertical.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void analogSetUp(){
        leftArmAnalog = myOpMode.hardwareMap.analogInput.get("left_analog");
        rightSwivelAnalog = myOpMode.hardwareMap.analogInput.get("right_swivel_analog");
        rightLinearAnalog = myOpMode.hardwareMap.analogInput.get("left_linear_analog");

        rightV4BEnc = new AbsoluteAnalogEncoder(leftArmAnalog, 3.3, 3);
        rightSwivelEnc = new AbsoluteAnalogEncoder(rightSwivelAnalog, 3.3, 21);
        rightLinearEnc = new AbsoluteAnalogEncoder(rightLinearAnalog, 3.3, 0);
    }

    public void driveRobot(double lf, double rf, double lb, double rb){
        // Send calculated power to wheels
        LF.setPower(lf);
        RF.setPower(rf);
        LB.setPower(lb);
        RB.setPower(rb);
    }
}

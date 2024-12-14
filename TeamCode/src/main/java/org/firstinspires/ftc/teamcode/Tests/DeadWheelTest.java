package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;

@TeleOp(name="DeadWheelTest", group="TeleOp")
public class DeadWheelTest extends LinearOpMode {

    public DcMotor LF;
    public DcMotor RF;
    public DcMotor LB;
    public DcMotor RB;
    public DcMotor topVertical;
    public DcMotor bottomVertical;
    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public Encoder strafeEncoder;


    @Override
    public void runOpMode(){
        LF = hardwareMap.get(DcMotor.class, "left_front");
        LB = hardwareMap.get(DcMotor.class, "left_back");
        RF = hardwareMap.get(DcMotor.class, "right_front");
        RB = hardwareMap.get(DcMotor.class, "right_back");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        topVertical = hardwareMap.get(DcMotor.class, "top_vertical");
        bottomVertical = hardwareMap.get(DcMotor.class, "bottom_vertical");

        topVertical.setDirection(DcMotorSimple.Direction.REVERSE);


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_front"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_back"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "top_vertical"));


    waitForStart();
    while (opModeIsActive()){
        telemetry.addData("Left Deadwheel Val: ", leftEncoder.getDeltaPosition());
        telemetry.addData("Right Deadwheel Val: ", rightEncoder.getDeltaPosition());
        telemetry.addData("Perp Deadwheel Val: ", strafeEncoder.getDeltaPosition());
        telemetry.update();
    }


    }
}

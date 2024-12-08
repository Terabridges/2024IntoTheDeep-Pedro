package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeSystem implements Subsystem {

    //Hardware
    public DcMotor topVertical;
    public DcMotor bottomVertical;
    public CRServo leftSwivel;
    public CRServo rightSwivel;
    public Servo wrist;
    public Servo claw;

    //Software

    //Constructor
    public OuttakeSystem(HardwareMap map) {
        topVertical = map.get(DcMotor.class, "top_vertical");
        bottomVertical = map.get(DcMotor.class, "bottom_vertical");
        leftSwivel = map.get(CRServo.class, "leftSwivel");
        rightSwivel = map.get(CRServo.class, "rightSwivel");
        wrist = map.get(Servo.class, "wrist");
        claw = map.get(Servo.class, "claw");
    }

    //Methods

    //Interface Methods
    @Override
    public void toInit(){
        //startupcode
    }

    @Override
    public void update(){
    }

}

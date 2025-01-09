package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

@Config
public class IntakeSystem implements Subsystem {

    //Hardware
    public CRServo intakeLeftSlide;
    public CRServo intakeRightSlide;
    public CRServo intakeRightSwivel;
    public CRServo intakeLeftSwivel;
    public CRServo intakeSpin;
    public AnalogInput intakeRightSwivelAnalog;
    public AnalogInput intakeRightLinearAnalog;
    public AbsoluteAnalogEncoder intakeRightSwivelEnc;
    public AbsoluteAnalogEncoder intakeRightLinearEnc;

    //Software
    public double RIGHT_SLIDE_RETRACT = 0.15;
    public double RIGHT_SLIDE_EXTEND = 0;
    public double LEFT_SLIDE_RETRACT = 0.85;
    public double LEFT_SLIDE_EXTEND = 1;
    public double INTAKE_PULL_IN = -0.4;
    public double INTAKE_PULL_OUT = 0.4;

    //Constructor
    public IntakeSystem(HardwareMap map) {
        intakeLeftSlide = map.get(CRServo.class, "intake_left_linear");
        intakeRightSlide = map.get(CRServo.class, "intake_right_linear");
        intakeLeftSwivel = map.get(CRServo.class, "intake_left_swivel");
        intakeRightSwivel = map.get(CRServo.class, "intake_right_swivel");
        intakeSpin = map.get(CRServo.class, "intake_spin");
        intakeRightSwivelAnalog = map.get(AnalogInput.class, "intake_right_swivel_analog");
        intakeRightLinearAnalog = map.get(AnalogInput.class, "intake_right_linear_analog");

    }

    //Methods
    public void intakeSetSlides(double lPos, double rPos) {

    }

    public void intakeSetSwivel(double lPos, double rPos) {

    }

    public void intakeSetSpin(double pow) {

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

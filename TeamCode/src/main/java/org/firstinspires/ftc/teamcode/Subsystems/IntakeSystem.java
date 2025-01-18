package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

@Config
public class IntakeSystem implements Subsystem {

    //Hardware
    private CRServo intakeLeftSlide;
    private CRServo intakeRightSlide;
    private CRServo intakeRightSwivel;
    private CRServo intakeLeftSwivel;
    private CRServo intakeSpin;
    private AnalogInput intakeRightSwivelAnalog;
    private AnalogInput intakeRightLinearAnalog;
    private AbsoluteAnalogEncoder intakeRightSwivelEnc;
    private AbsoluteAnalogEncoder intakeRightLinearEnc;

    //SOFTWARE
    //Positions
    private double INTAKE_SPIN_IN = -0.4;
    private double INTAKE_SPIN_OUT = 0.4;
    private double INTAKE_SPIN_STOP = 0;
    private int INTAKE_SLIDES_EXTEND = 227;
    private int INTAKE_SLIDES_RETRACT = 190;
    private int INTAKE_SWIVEL_TRANSFER;
    private int INTAKE_SWIVEL_DOWN;
    private int INTAKE_SWIVEL_REST;

    //Targets
    private int intakeSlidesTarget;
    private int intakeSwivelTarget;
    private double intakeSpinTarget = 0;

    //Max
    private double INTAKE_SLIDES_MAX_POWER;;
    private double INTAKE_SWIVEL_MAX_POWER;
    private double INTAKE_SPIN_MAX_POWER;

    //PIDF



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

    //METHODS
    //Core Methods
    public void intakeSetSlides(double pow) {
        if(pow > INTAKE_SLIDES_MAX_POWER) pow = INTAKE_SLIDES_MAX_POWER;
        intakeLeftSlide.setPower(pow);
        intakeRightSlide.setPower(pow);
    }

    public void intakeSetSwivel(double pow) {
        if(pow > INTAKE_SWIVEL_MAX_POWER) pow = INTAKE_SWIVEL_MAX_POWER;
        intakeLeftSwivel.setPower(pow);
        intakeRightSwivel.setPower(pow);
    }

    public void intakeSetSpin(double pow) {
        if(pow > INTAKE_SPIN_MAX_POWER) pow = INTAKE_SPIN_MAX_POWER;
        intakeSpin.setPower(pow);
    }

    //Other Methods
    public void intakeSlidesExtend() {
        intakeSlidesTarget = INTAKE_SLIDES_EXTEND;
    }

    public void intakeSlidesRetract() {
        intakeSlidesTarget = INTAKE_SLIDES_RETRACT;
    }

    public void intakeSwivelRest() {
        intakeSwivelTarget = INTAKE_SWIVEL_REST;
    }

    public void intakeSwivelDown(){
        intakeSwivelTarget = INTAKE_SWIVEL_DOWN;
    }

    public void intakeSwivelTransfer(){
        intakeSwivelTarget = INTAKE_SWIVEL_TRANSFER;
    }

    public void intakeSpinIn() {
        intakeSpinTarget = INTAKE_SPIN_IN;
    }

    public void intakeSpinOut() {
        intakeSpinTarget = INTAKE_SPIN_OUT;
    }

    public void intakeStopSpin() {
        intakeSpinTarget = INTAKE_SPIN_STOP;
    }

    //PIDF
    private int setIntakeSlidesPIDF(int target) {
        return 0;
    }

    private int setIntakeSwivelPIDF(int target) {
        return 0;
    }

    //Interface Methods
    @Override
    public void toInit(){
        intakeSlidesRetract();
        intakeSwivelRest();
        intakeStopSpin();
    }

    @Override
    public void update(){
        intakeSetSlides(setIntakeSlidesPIDF(intakeSlidesTarget));
        intakeSetSwivel(setIntakeSwivelPIDF(intakeSwivelTarget));
        intakeSetSpin(intakeSpinTarget);
    }

}

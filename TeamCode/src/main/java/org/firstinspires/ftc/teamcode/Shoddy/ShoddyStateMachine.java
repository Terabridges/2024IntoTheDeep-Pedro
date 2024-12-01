package org.firstinspires.ftc.teamcode.Shoddy;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShoddyStateMachine {

    private LinearOpMode myOpMode;
    public ShoddyStateMachine(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    //                          PIDF STUFF
    //First PID for V4B
    private PIDController controller;
    public static double p = 0.005, i = 0.01, d = 0.00004;
    public static double f = 0.06;
    private final double ticks_in_degree = 144.0 / 180.0;
    public static int V4BTarget;
    double armPos;
    double pid, targetArmAngle, ff, currentArmAngle, V4BPower;

    //Second PID for Vertical Slides
    private PIDController controller2;
    public static double p2 = 0.006, i2 = 0.001, d2 = 0;
    public static double f2 = 0;
    private final double ticks_in_degree2 = 144.0 / 180.0;
    public static int vertSlidesTarget;
    double armPos2;
    double pid2, targetArmAngle2, ff2, currentArmAngle2, verticalSlidesPower;

    //Third PID for Swivel
    private PIDController controller3;
    public static double p3 = -0.005, i3 = 0.01, d3 = 0.0002;
    public static double f3 = 0.035;
    private final double ticks_in_degree3 = 144.0 / 180.0;
    public static int swivelTarget;
    double armPos3;
    double pid3, targetArmAngle3, ff3, currentArmAngle3, swivelPower;



    //                      OTHER SHIT
    //TODO fix the null reference
    ShoddyRobotClass r = new ShoddyRobotClass(myOpMode);
    ShoddyPositions po = new ShoddyPositions();
    ShoddyToggles t = new ShoddyToggles(myOpMode);

    int extendTime = 500;
    int intakeTime = 2000;
    int dropTime = 500;

    //Right stick button is cancel for all macros
    //There are three main macros: intake, outtake, and transfer



    //                              INTAKE STATES
    // INTAKE_START: intake is in default position in which linear slides are in and V4B is in rest position
    //      once button pressed:
    // INTAKE_EXTEND: linear slides come out to desired position
    //      once linear slides position is close to desired: (no encoder so delay)
    // INTAKE_GRAB: V4B drops down
    //      after a button:
    // INTAKE_RETRACT: intake stops spinning, V4B comes up to resting position, linear slides come in
    //      once linear slides are in, go back to intake start (no encoder so delay)

    public enum IntakeState {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_GRAB,
        INTAKE_RETRACT
    };

    public IntakeState intakeState = IntakeState.INTAKE_START;
    public ElapsedTime intakeTimer = new ElapsedTime();

    public void intakeStateCheck() {
        switch (intakeState) {
            case INTAKE_START:
                if (t.currentGamepad1.a && !t.previousGamepad1.a){
                    r.leftLinear.setPosition(po.LEFT_SLIDE_OUT);
                    r.rightLinear.setPosition(po.RIGHT_SLIDE_OUT);
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_EXTEND;
                }
                break;
            case INTAKE_EXTEND:
                if (intakeTimer.milliseconds() >= extendTime) {
                    setV4BPIDF(po.V4B_INTAKE_POS);
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_GRAB;
                }
                break;
            case INTAKE_GRAB:
                if (t.currentGamepad1.a && !t.previousGamepad1.a) {
                    r.intake.setPower(0);
                    setV4BPIDF(po.V4B_REST_POS);
                    r.leftLinear.setPosition(po.LEFT_SLIDE_IN);
                    r.rightLinear.setPosition(po.RIGHT_SLIDE_IN);
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_RETRACT;
                }
                break;
            case INTAKE_RETRACT:
                if (intakeTimer.milliseconds() >= extendTime) {
                    intakeState = IntakeState.INTAKE_START;
                }
                break;
            default:
                intakeState = IntakeState.INTAKE_START;

        }

        if ((t.currentGamepad1.right_stick_button && !t.previousGamepad1.right_stick_button) && (intakeState != IntakeState.INTAKE_START)){
            intakeState = IntakeState.INTAKE_START;
        }
    }



    //                              OUTTAKE STATES
    // OUTTAKE_START: outtake is in default position in which swivel is down, claw is closed, and vertical slides are in rest position
    //      once button pressed:
    // OUTTAKE_EXTEND: vertical slides extend to high position
    //      once vertical slides rise to desired position:
    // OUTTAKE_SWIVEL: swivel flips up
    //      once button pressed:
    // OUTTAKE_DROP: claw opens
    //      after a delay:
    // OUTTAKE_RETRACT: claw closes, swivel comes down, vertical slides fall to rest position
    //      once vertical slides are down, go back to outtake start

    public enum OuttakeState {
        OUTTAKE_START,
        OUTTAKE_EXTEND,
        OUTTAKE_SWIVEL,
        OUTTAKE_DROP,
        OUTTAKE_RETRACT
    };

    public OuttakeState outtakeState = OuttakeState.OUTTAKE_START;
    public ElapsedTime outtakeTimer = new ElapsedTime();

    public void outtakeStateCheck() {
        switch (outtakeState) {
            case OUTTAKE_START:
                if (t.currentGamepad1.y && !t.previousGamepad1.y){
                    setVerticalSlidesPIDF(po.VERTICAL_UP);
                    outtakeState = OuttakeState.OUTTAKE_EXTEND;
                }
                break;
            case OUTTAKE_EXTEND:
                if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_UP) < 50) {
                    setSwivelPIDF(po.SWIVEL_UP);
                    outtakeState = OuttakeState.OUTTAKE_SWIVEL;
                }
                break;
            case OUTTAKE_SWIVEL:
                if (t.currentGamepad1.y && !t.previousGamepad1.y) {
                    r.claw.setPosition(po.CLAW_OPEN);
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.OUTTAKE_DROP;
                }
                break;
            case OUTTAKE_DROP:
                if (outtakeTimer.milliseconds() >= dropTime) {
                    r.claw.setPosition(po.CLAW_CLOSED);
                    setSwivelPIDF(po.SWIVEL_DOWN);
                    setVerticalSlidesPIDF(po.VERTICAL_REST);
                    outtakeState = OuttakeState.OUTTAKE_RETRACT;
                }
                break;
            case OUTTAKE_RETRACT:
                if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_REST) < 50) {
                    outtakeState = OuttakeState.OUTTAKE_START;

                }
                break;
            default:
                outtakeState = OuttakeState.OUTTAKE_START;
        }

        if ((t.currentGamepad1.right_stick_button && !t.previousGamepad1.right_stick_button) && (outtakeState != OuttakeState.OUTTAKE_START)){
            outtakeState = OuttakeState.OUTTAKE_START;
        }
    }

    //                            TRANSFER STATES
    // TRANSFER_START: intake is in default position, outtake is in default position
    //      once button pressed:
    // TRANSFER_INTAKE: V4B moves to transfer position, linear slides move in all the way
    //      once V4B is in transfer position:
    // TRANSFER_CLAW: claw opens, vertical slides move to transport position
    //      once vertical slides are in position:
    // TRANSFER_OUTTAKE: claw closes, vertical slides move to rest position
    //      once vertical slides are in rest position, go back to transfer start

    public enum TransferState {
        TRANSFER_START,
        TRANSFER_INTAKE,
        TRANSFER_CLAW,
        TRANSFER_OUTTAKE
    };

    public TransferState transferState = TransferState.TRANSFER_START;
    public ElapsedTime transferTimer = new ElapsedTime();

    public void transferStateCheck() {
        switch (transferState) {
            case TRANSFER_START:
                if (t.currentGamepad1.b && !t.previousGamepad1.b){
                    r.leftLinear.setPosition(po.LEFT_SLIDE_IN);
                    r.rightLinear.setPosition(po.RIGHT_SLIDE_IN);
                    setV4BPIDF(po.V4B_TRANSFER_POS);
                    transferState = TransferState.TRANSFER_INTAKE;
                }
                break;
            case TRANSFER_INTAKE:
                if (Math.abs(r.rightSwivelEnc.getCurrentPosition() - po.V4B_TRANSFER_POS) < 10) {
                    r.claw.setPosition(po.CLAW_OPEN);
                    setVerticalSlidesPIDF(po.VERTICAL_DOWN);
                    transferState = TransferState.TRANSFER_CLAW;
                }
                break;
            case TRANSFER_CLAW:
                if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_DOWN) < 50) {
                    r.claw.setPosition(po.CLAW_CLOSED);
                    setVerticalSlidesPIDF(po.VERTICAL_REST);
                    transferState = TransferState.TRANSFER_OUTTAKE;
                }
                break;
            case TRANSFER_OUTTAKE:
                if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_REST) < 50) {
                    transferState = TransferState.TRANSFER_START;
                }
                break;
            default:
                transferState = TransferState.TRANSFER_START;
        }

        if ((t.currentGamepad1.right_stick_button && !t.previousGamepad1.right_stick_button) && (transferState != TransferState.TRANSFER_START)){
            transferState = TransferState.TRANSFER_START;
        }
    }


    //                      PIDF METHODS
    public void setV4BPIDF(int target) {
        controller.setPID(p, i, d);
        armPos = r.rightV4BEnc.getCurrentPosition();
        pid = controller.calculate(armPos, target);
        targetArmAngle = target;
        ff = (Math.sin(Math.toRadians(targetArmAngle))) * f;
        currentArmAngle = Math.toRadians((armPos) / ticks_in_degree);

        V4BPower = pid + ff;

        r.leftArm.setPower(V4BPower);
        r.rightArm.setPower(V4BPower);
    }

    public void setVerticalSlidesPIDF(int target2) {
        controller2.setPID(p2, i2, d2);
        armPos2 = r.topVertical.getCurrentPosition();
        pid2 = controller2.calculate(armPos2, target2);
        targetArmAngle2 = Math.toRadians((target2) / ticks_in_degree2);
        ff2 = targetArmAngle2 * f2;
        currentArmAngle2 = Math.toRadians((armPos2) / ticks_in_degree2);

        verticalSlidesPower = pid2 + ff2;

        r.topVertical.setPower(verticalSlidesPower);
        r.bottomVertical.setPower(verticalSlidesPower);
    }

    public void setSwivelPIDF(int target3) {
        controller3.setPID(p3, i3, d3);
        armPos3 = r.rightSwivelEnc.getCurrentPosition();
        pid3 = controller3.calculate(armPos3, target3);
        targetArmAngle3 = target3;
        ff3 = (Math.cos(Math.toRadians(targetArmAngle3))) * f3;
        currentArmAngle3 = Math.toRadians((armPos3) / ticks_in_degree3);

        swivelPower = pid3 + ff3;

        r.leftSwivel.setPower(swivelPower);
        r.rightSwivel.setPower(swivelPower);
    }

}

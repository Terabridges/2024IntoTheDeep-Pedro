package org.firstinspires.ftc.teamcode.Shoddy;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.FtcDashboard;


//TODO slow mode (left bumper), assign x button and (dpad down probably should move vertical slides to specimen position), figure out wrist positions, figure out swivel level, maybe do field centric toggle?

@Config
@TeleOp(name="ShoddyTeleOp", group="TeleOp")
public class ShoddyTeleOp extends LinearOpMode {

    //public DcMotor verticalEnc;

    ShoddyRobotClass r;
    ShoddyToggles t;
    ShoddyPositions po;
    private ElapsedTime runtime;


    public boolean usePIDFvertical = true;
    public boolean usePIDFswivel = true;
    public boolean usePIDFV4B = true;

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
    public static double p3 = -0.006, i3 = 0.02, d3 = 0.0002;
    public static double f3 = 0.035;
    private final double ticks_in_degree3 = 144.0 / 180.0;
    public static int swivelTarget;
    double armPos3;
    double pid3, targetArmAngle3, ff3, currentArmAngle3, swivelPower;

    //FSM TEST STUFF
    int extendTime = 400;
    int transferTime = 2000;
    int dropTime = 1000;

    public enum IntakeState {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_GRAB,
        INTAKE_RETRACT
    };

    public enum OuttakeState {
        OUTTAKE_START,
        OUTTAKE_EXTEND,
        OUTTAKE_SWIVEL,
        OUTTAKE_DROP,
        OUTTAKE_RETRACT
    };

    public enum TransferState {
        TRANSFER_START,
        TRANSFER_INTAKE,
        TRANSFER_CLAW,
        TRANSFER_OUTTAKE
    };

    public IntakeState intakeState = IntakeState.INTAKE_START;
    public ElapsedTime intakeTimer = new ElapsedTime();
    public OuttakeState outtakeState = OuttakeState.OUTTAKE_START;
    public ElapsedTime outtakeTimer = new ElapsedTime();
    public TransferState transferState = TransferState.TRANSFER_START;
    public ElapsedTime transferTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        r = new ShoddyRobotClass(this);
        t = new ShoddyToggles(this);
        po = new ShoddyPositions();
        runtime = new ElapsedTime();

        r.wheelSetUp();
        r.servoSetUp();
        r.motorSetUp();
        r.analogSetUp();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);
        controller3 = new PIDController(p3, i3, d3);

        r.topVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        V4BTarget = po.V4B_TRANSFER_POS;
        vertSlidesTarget = po.VERTICAL_REST;
        swivelTarget = po.SWIVEL_DOWN;

        double leftLinearTarget = po.LEFT_SLIDE_IN;
        double rightLinearTarget = po.RIGHT_SLIDE_IN;
        double clawTarget = po.CLAW_CLOSED;
        double wristTarget = po.WRIST_PAR;

        double botVerticalPower = 0;
        double topVerticalPower = 0;
        double intakePower = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            t.copyGamepad();

            //INIT

            //Robot Drive (Left Stick and Right Stick X)
            {
                double max;
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;
                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));
                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                leftFrontPower *= po.speed;
                rightFrontPower *= po.speed;
                leftBackPower *= po.speed;
                leftFrontPower *= po.speed;

                // Send calculated power to wheels
                r.driveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }

            // Intake Power Toggles (DPAD RIGHT and DPAD LEFT)
            {
                if (t.toggle("dpad_right")) {
                    if (t.dpRightToggle) {
                        intakePower = po.INTAKE_POWER_IN;
                    } else {
                        intakePower = 0;
                    }
                }

                if (t.toggle("dpad_left")) {
                    if (t.dpLeftToggle) {
                        intakePower = po.INTAKE_POWER_OUT;
                    } else {
                        intakePower = 0;
                    }
                }
            }

            //Claw Open/Closed (Right Bumper)
            {

                if (t.toggle("right_bumper")) {
                    if (t.rBumpToggle) {
                        clawTarget = po.CLAW_OPEN;
                    } else {
                        clawTarget = po.CLAW_CLOSED;
                    }
                }
            }

            // Vertical Adjust (Triggers) Vertical Hold (Left stick button)
            {
                if ((t.currentGamepad1.right_trigger != 0) && !(t.previousGamepad1.right_trigger != 0)){
                    usePIDFvertical = false;
                    botVerticalPower = gamepad1.right_trigger;
                    topVerticalPower = gamepad1.right_trigger;
                }

                if ((t.currentGamepad1.left_trigger != 0) && !(t.previousGamepad1.left_trigger != 0)){
                    usePIDFvertical = false;
                    botVerticalPower = -gamepad1.left_trigger;
                    topVerticalPower = -gamepad1.left_trigger;
                }

                if (t.currentGamepad1.left_stick_button && !t.previousGamepad1.left_stick_button){
                    usePIDFvertical = true;
                    vertSlidesTarget = r.topVertical.getCurrentPosition();
                }
            }

            //Auto Intake (A)
            {
                switch (intakeState) {
                    case INTAKE_START:
                        if (t.currentGamepad1.a && !t.previousGamepad1.a){
                            usePIDFvertical = true;
                            leftLinearTarget = po.LEFT_SLIDE_OUT;
                            rightLinearTarget = po.RIGHT_SLIDE_OUT;
                            intakeTimer.reset();
                            intakeState = IntakeState.INTAKE_EXTEND;
                        }
                        break;
                    case INTAKE_EXTEND:
                        if (intakeTimer.milliseconds() >= extendTime) {
                            V4BTarget = po.V4B_INTAKE_POS;
                            intakeTimer.reset();
                            intakeState = IntakeState.INTAKE_GRAB;
                        }
                        break;
                    case INTAKE_GRAB:
                        if (t.currentGamepad1.a && !t.previousGamepad1.a) {
                            V4BTarget = po.V4B_REST_POS;
                            leftLinearTarget = po.LEFT_SLIDE_IN;
                            rightLinearTarget = po.RIGHT_SLIDE_IN;
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

            //Auto Outtake (Y)
            {
                switch (outtakeState) {
                    case OUTTAKE_START:
                        if (t.currentGamepad1.y && !t.previousGamepad1.y){
                            usePIDFvertical = true;
                            vertSlidesTarget = po.VERTICAL_UP;
                            outtakeState = OuttakeState.OUTTAKE_EXTEND;
                        }
                        break;
                    case OUTTAKE_EXTEND:
                        if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_UP) < 50) {
                            swivelTarget = po.SWIVEL_UP;
                            outtakeState = OuttakeState.OUTTAKE_SWIVEL;
                        }
                        break;
                    case OUTTAKE_SWIVEL:
                        if (t.currentGamepad1.y && !t.previousGamepad1.y) {
                            clawTarget = po.CLAW_OPEN;
                            outtakeTimer.reset();
                            outtakeState = OuttakeState.OUTTAKE_DROP;
                        }
                        break;
                    case OUTTAKE_DROP:
                        if ((t.currentGamepad1.y && !t.previousGamepad1.y) && (outtakeTimer.milliseconds() >= 1000)) {
                            clawTarget = po.CLAW_CLOSED;
                            swivelTarget = po.SWIVEL_DOWN;
                            vertSlidesTarget = po.VERTICAL_REST;
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

            //Auto Transfer (B)
            {
                switch (transferState) {
                    case TRANSFER_START:
                        if (t.currentGamepad1.b && !t.previousGamepad1.b){
                            usePIDFvertical = true;
                            leftLinearTarget = po.LEFT_SLIDE_IN;
                            rightLinearTarget = po.RIGHT_SLIDE_IN;
                            V4BTarget = po.V4B_TRANSFER_POS;
                            transferState = TransferState.TRANSFER_INTAKE;
                        }
                        break;
                    case TRANSFER_INTAKE:
                        if (Math.abs(r.rightV4BEnc.getCurrentPosition() - po.V4B_TRANSFER_POS) < 10) {
                            clawTarget = po.CLAW_OPEN;
                            vertSlidesTarget = po.VERTICAL_DOWN;
                            transferState = TransferState.TRANSFER_CLAW;
                        }
                        break;
                    case TRANSFER_CLAW:
                        if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_DOWN) < 50) {
                            clawTarget = po.CLAW_CLOSED;
                            vertSlidesTarget = po.VERTICAL_REST;
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

            // Slow Mode Toggle (Left Bumper)
            {
                if (t.toggle("left_bumper")) {
                    if (t.lBumpToggle) {
                        po.speed = po.speed * po.slowMultiplier;
                    } else {
                        po.speed = po.maxSpeed;
                    }
                }
            }

            //Set Swivel level toggle for specimen (dpad up)
            {
                if (t.toggle("dpad_up")) {
                    if (t.dpUpToggle) {
                        swivelTarget = po.SWIVEL_LEVEL;
                        wristTarget = po.WRIST_PERP;
                    } else {
                        swivelTarget = po.SWIVEL_DOWN;
                        wristTarget = po.WRIST_PAR;
                    }
                }
            }

            //Set powers
            setV4BPIDF(V4BTarget);
            setSwivelPIDF(swivelTarget);

            if (usePIDFvertical) {
                setVerticalSlidesPIDF(vertSlidesTarget);
            } else {
                r.bottomVertical.setPower(botVerticalPower);
                r.topVertical.setPower(topVerticalPower);
            }

            r.rightLinear.setPosition(rightLinearTarget);
            r.leftLinear.setPosition(leftLinearTarget);
            r.claw.setPosition(clawTarget);
            r.wrist.setPosition(wristTarget);
            r.intake.setPower(intakePower);


            //Telemetry
            telemetry.addData("runtime", runtime.milliseconds());
            telemetry.addData("V4B Pos", r.rightV4BEnc.getCurrentPosition());
            telemetry.addData("V4B Target", V4BTarget);
            telemetry.addData("Swivel Pos", r.rightSwivelEnc.getCurrentPosition());
            telemetry.addData("Swivel Target", swivelTarget);
            telemetry.addData("Vert Pos", r.topVertical.getCurrentPosition());
            telemetry.addData("Vert Target", vertSlidesTarget);
            telemetry.addData("Left Slide", leftLinearTarget);
            telemetry.addData("Right Slide", rightLinearTarget);
            telemetry.update();

        }
    }
    // Methods
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


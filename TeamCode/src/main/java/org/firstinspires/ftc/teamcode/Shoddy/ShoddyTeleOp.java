package org.firstinspires.ftc.teamcode.Shoddy;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.FtcDashboard;

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
    public static double p = 0.005, i = 0.02, d = 0.00004;
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

    public enum SpecimenState {
        SPECIMEN_START,
        SPECIMEN_MOVE,
        SPECIMEN_WALL,
        SPECIMEN_RISE,
        SPECIMEN_FALL,
        SPECIMEN_RESET
    }

    public IntakeState intakeState = IntakeState.INTAKE_START;
    public ElapsedTime intakeTimer = new ElapsedTime();
    public OuttakeState outtakeState = OuttakeState.OUTTAKE_START;
    public ElapsedTime outtakeTimer = new ElapsedTime();
    public TransferState transferState = TransferState.TRANSFER_START;
    public ElapsedTime transferTimer = new ElapsedTime();
    public SpecimenState specimenState = SpecimenState.SPECIMEN_START;
    public ElapsedTime specimenTimer = new ElapsedTime();

    public double distanceOutput = 0;
    public boolean useReduceSpeed = false;
    public double defaultDistanceReduce = 10;
    public double defaultDistanceStop = 5;


    /////////////////////////////////////////////////////////////////////////////////////////
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

        //TODO Turn off for competitions
        r.topVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double leftLinearTarget;
        double rightLinearTarget;
        double clawTarget;
        double wristTarget;

        double botVerticalPower;
        double topVerticalPower;
        double intakePower;

        boolean triggerIntake = true;
        boolean resetState = false;
        boolean manualPower = true;

        boolean encodersReset = false;

        po.speed = po.fastSpeed;


        ////////////////////////////////////////////////////////////////////////////////
//        while (!isStarted()) {
//            if (gamepad1.dpad_up) {
//                r.topVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                r.bottomVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                encodersReset = true;
//            }
//            telemetry.addData("Encoders Reset? ", encodersReset);
//            telemetry.update();
//        }

        ////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        runtime.reset();

        V4BTarget = po.V4B_TRANSFER_POS;
        vertSlidesTarget = po.VERTICAL_REST;
        swivelTarget = po.SWIVEL_DOWN;

        leftLinearTarget = po.LEFT_SLIDE_IN;
        rightLinearTarget = po.RIGHT_SLIDE_IN;
        clawTarget = po.CLAW_CLOSED;
        wristTarget = po.WRIST_PAR;

        botVerticalPower = 0;
        topVerticalPower = 0;
        intakePower = 0;

        while (opModeIsActive()) {

            t.copyGamepad();

            //INIT

            distanceOutput = (r.distanceSensorRear.getVoltage()*48.7)-4.9;

            //Robot Drive (Left Stick and Right Stick X)
            {
                double max;
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x * po.TURN_VAL;
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
                rightBackPower *= po.speed;

                // Send calculated power to wheels
                if (useReduceSpeed){
                    r.driveRobot(reducePower(leftFrontPower), reducePower(rightFrontPower), reducePower(leftBackPower), reducePower(rightBackPower));

                } else if (manualPower) {
                    r.driveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
                }
            }

            // Use Reduce Speed Mode (dpad left)
            {
//                if (t.toggle("dpad_right")) {
//                    if (t.dpRightToggle) {
//                        intakePower = po.INTAKE_POWER_IN;
//                    } else {
//                        intakePower = 0;
//                    }
//                }

                if (t.toggle("dpad_left")) {
                    if (t.dpLeftToggle) {
                        useReduceSpeed = !useReduceSpeed;
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

            //Sam vs Rohan mode (Right Stick Button)
            {
                if (t.toggle("right_stick_button")){
                    if (t.rStickToggle){
                        po.ROHAN_MODE = true;
                    } else {
                        po.ROHAN_MODE = false;
                    }
                }
            }

            // Vertical Adjust (Triggers) Vertical Hold (Left stick button)
            {

                if (!triggerIntake) {
                    if (t.currentGamepad1.right_trigger > po.MIN_TRIGGER_VAL) {
                        usePIDFvertical = false;
                        botVerticalPower = gamepad1.right_trigger;
                        topVerticalPower = gamepad1.right_trigger;
                    } else if (t.currentGamepad1.left_trigger > po.MIN_TRIGGER_VAL) {
                        usePIDFvertical = false;
                        botVerticalPower = -gamepad1.left_trigger;
                        topVerticalPower = -gamepad1.left_trigger;
                    }

                    if ((t.currentGamepad1.right_trigger == 0) && (t.currentGamepad1.left_trigger == 0) && (!usePIDFvertical)) {
                        usePIDFvertical = true;
                        vertSlidesTarget = r.topVertical.getCurrentPosition();
                    }
                } else {
                    if (t.currentGamepad1.right_trigger > po.MIN_TRIGGER_VAL) {
                        intakePower = t.currentGamepad1.right_trigger;
                    } else if (t.currentGamepad1.left_trigger > po.MIN_TRIGGER_VAL) {
                        intakePower = -t.currentGamepad1.left_trigger;
                    } else {
                        intakePower = 0;
                    }
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
                            if (po.ROHAN_MODE) {
                                r.LB.setDirection(DcMotorSimple.Direction.FORWARD);
                                r.RB.setDirection(DcMotorSimple.Direction.REVERSE);
                                r.LF.setDirection(DcMotorSimple.Direction.FORWARD);
                                r.RF.setDirection(DcMotorSimple.Direction.REVERSE);
                                po.TURN_VAL = -1;
                                po.reversed = true;
                            }
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
            }

            //Auto Outtake (Y)
            {
                switch (outtakeState) {
                    case OUTTAKE_START:
                        if (t.currentGamepad1.y && !t.previousGamepad1.y){
                            usePIDFvertical = true;
                            vertSlidesTarget = po.VERTICAL_UP;
                            swivelTarget = po.SWIVEL_UP;
                            wristTarget = po.WRIST_PERP;
                            outtakeState = OuttakeState.OUTTAKE_EXTEND;
                        }
                        break;
                    case OUTTAKE_EXTEND:
                            outtakeState = OuttakeState.OUTTAKE_SWIVEL;
                        break;
                    case OUTTAKE_SWIVEL:
                        if (t.currentGamepad1.y && !t.previousGamepad1.y) {
                            clawTarget = po.CLAW_OPEN;
                            if (po.ROHAN_MODE) {
                                r.LB.setDirection(DcMotorSimple.Direction.REVERSE);
                                r.RB.setDirection(DcMotorSimple.Direction.FORWARD);
                                r.LF.setDirection(DcMotorSimple.Direction.REVERSE);
                                r.RF.setDirection(DcMotorSimple.Direction.FORWARD);
                                po.TURN_VAL = 1;
                                po.reversed = false;
                            }

                            manualPower = false;
                            r.LB.setPower(0.5);
                            r.LF.setPower(0.5);
                            r.RB.setPower(0.5);
                            r.RF.setPower(0.5);

                            outtakeTimer.reset();
                            outtakeState = OuttakeState.OUTTAKE_DROP;
                        }
                        break;
                    case OUTTAKE_DROP:
                        if (outtakeTimer.milliseconds() >= 500) {
                            manualPower = true;
                            r.LB.setPower(0);
                            r.LF.setPower(0);
                            r.RB.setPower(0);
                            r.RF.setPower(0);
                            clawTarget = po.CLAW_CLOSED;
                            wristTarget = po.WRIST_PAR;
                            swivelTarget = po.SWIVEL_DOWN;
                            vertSlidesTarget = po.VERTICAL_REST;
                            outtakeState = OuttakeState.OUTTAKE_RETRACT;
                        }
                        break;
                    case OUTTAKE_RETRACT:
                        if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_REST) <= 50) {
                            outtakeState = OuttakeState.OUTTAKE_START;

                        }
                        break;
                    default:
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
                        if (Math.abs(r.rightV4BEnc.getCurrentPosition() - po.V4B_TRANSFER_POS) <= 10) {
                            clawTarget = po.CLAW_OPEN;
                            vertSlidesTarget = po.VERTICAL_DOWN;
                            transferState = TransferState.TRANSFER_CLAW;
                        }
                        break;
                    case TRANSFER_CLAW:
                        if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_DOWN) <= 50) {
                            clawTarget = po.CLAW_CLOSED;
                            vertSlidesTarget = po.VERTICAL_REST;
                            transferState = TransferState.TRANSFER_OUTTAKE;
                        }
                        break;
                    case TRANSFER_OUTTAKE:
                        if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_REST) <= 50) {
                            transferState = TransferState.TRANSFER_START;
                        }
                        break;
                    default:
                        transferState = TransferState.TRANSFER_START;
                }
            }

            //Swap Direction (X)
            {
                if (t.toggle("x")) {
                    if (po.ROHAN_MODE) {
                        if (t.xToggle) {
                            //Reverse
                            r.LB.setDirection(DcMotorSimple.Direction.FORWARD);
                            r.RB.setDirection(DcMotorSimple.Direction.REVERSE);
                            r.LF.setDirection(DcMotorSimple.Direction.FORWARD);
                            r.RF.setDirection(DcMotorSimple.Direction.REVERSE);
                            po.TURN_VAL = -1;
                            po.reversed = true;
                        } else {
                            //Normal
                            r.LB.setDirection(DcMotorSimple.Direction.REVERSE);
                            r.RB.setDirection(DcMotorSimple.Direction.FORWARD);
                            r.LF.setDirection(DcMotorSimple.Direction.REVERSE);
                            r.RF.setDirection(DcMotorSimple.Direction.FORWARD);
                            po.TURN_VAL = 1;
                            po.reversed = false;
                        }
                    }
                }
            }


            // Slow Mode Toggle (Left Bumper)
            {
                if (t.toggle("left_bumper")) {
                    if (po.ROHAN_MODE) {
                        if (t.lBumpToggle) {
                            po.speed = po.slowSpeed;
                        } else {
                            po.speed = po.fastSpeed;
                        }
                    } else {
                        if (t.lBumpToggle) {
                            po.speed = po.samSlowSpeed;
                        } else {
                            po.speed = po.fastSpeed;
                        }
                    }
                }
            }

            //Switch Trigger Mode (Left Stick Button)
            {
                if (t.toggle("left_stick_button")) {
                    if (t.lStickToggle) {
                        triggerIntake = false;
                    } else {
                        triggerIntake = true;
                    }
                }
            }


            //Nothing currently (dpad up)
            {

            }

            //Set Swivel for wall grab
            {
                switch (specimenState){
                    case SPECIMEN_START:
                        if (t.currentGamepad1.dpad_down && !t.previousGamepad1.dpad_down){
                            usePIDFvertical = true;
                            clawTarget = po.CLAW_OPEN;
                            wristTarget = po.WRIST_PERP;
                            swivelTarget = po.SWIVEL_WALL;
                            vertSlidesTarget = po.VERTICAL_DOWN;
                            specimenTimer.reset();
                            specimenState = SpecimenState.SPECIMEN_WALL;
                        }
                        break;
                    case SPECIMEN_WALL:
                        if (t.currentGamepad1.dpad_down && !t.previousGamepad1.dpad_down && specimenTimer.milliseconds() >= 300){
                            clawTarget = po.CLAW_CLOSED;
                            specimenTimer.reset();
                            specimenState = SpecimenState.SPECIMEN_RISE;
                        }
                        break;
                    case SPECIMEN_RISE:
                        if (t.currentGamepad1.dpad_down && !t.previousGamepad1.dpad_down && specimenTimer.milliseconds() >= 300){
                            vertSlidesTarget = po.VERTICAL_BAR_UP;
                            swivelTarget = po.SWIVEL_BAR;
                            specimenState = SpecimenState.SPECIMEN_MOVE;
                        }
                        break;
                    case SPECIMEN_MOVE:
                        if (t.currentGamepad1.dpad_down && !t.previousGamepad1.dpad_down) {
                            manualPower = false;
                            r.LB.setPower(-0.3);
                            r.LF.setPower(-0.3);
                            r.RB.setPower(-0.3);
                            r.RF.setPower(-0.3);
                            specimenState = SpecimenState.SPECIMEN_FALL;
                        }
                        break;
                    case SPECIMEN_FALL:
                        if (distanceOutput <= po.specimenDistance){
                            manualPower = true;
                            r.LB.setPower(0);
                            r.LF.setPower(0);
                            r.RB.setPower(0);
                            r.RF.setPower(0);
                            //vertSlidesTarget = po.VERTICAL_BAR_DOWN;
                            //specimenState = SpecimenState.SPECIMEN_RESET;
                            specimenState = SpecimenState.SPECIMEN_START;
                        }
                        break;
                    case SPECIMEN_RESET:
                        if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_BAR_DOWN) < 50){
                            clawTarget = po.CLAW_OPEN;
                            wristTarget = po.WRIST_PAR;
                            swivelTarget = po.SWIVEL_DOWN;
                            vertSlidesTarget = po.VERTICAL_BAR_DOWN;
                            specimenState = SpecimenState.SPECIMEN_START;
                        }
                        break;
                    default:
                        specimenState = SpecimenState.SPECIMEN_START;
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
            telemetry.addData("Reversed: ", po.reversed);
            telemetry.addData("Current Speed: ", po.speed);
            telemetry.addData("Distance: ", distanceOutput);
            telemetry.addData("Reduce Speed Mode: ", useReduceSpeed);

            if (po.ROHAN_MODE){
                telemetry.addData("Driver: ", "Saucy Indian Boy");
            } else {
                telemetry.addData("Driver: ", "My Asian Superstar <3");
            }
            if (runtime.seconds() == 10){
                gamepad1.rumble(500);
            }

            if (useReduceSpeed){
                telemetry.addData("If 100% Power: ", reducePower(1));
            }

            telemetry.update();

        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

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

    public double reducePower(double power, double distance, double distanceStartReducing, double distanceToStop){
        double powerFactor;
        double minPowerFactor = 0.05;
        if ((distance <= distanceStartReducing) && (distance > distanceToStop)){
            powerFactor = ((distance - distanceToStop) / (distanceStartReducing - distanceToStop))/2;
        } else if (distance <= distanceToStop){
            powerFactor = 0;
        } else {
            powerFactor = 1;
        }
        if (powerFactor < minPowerFactor){
            powerFactor = 0.1;
        }
        return power*powerFactor;
    }

    public double reducePower(double power){
        return this.reducePower(power, distanceOutput, defaultDistanceReduce, defaultDistanceStop);
    }
}


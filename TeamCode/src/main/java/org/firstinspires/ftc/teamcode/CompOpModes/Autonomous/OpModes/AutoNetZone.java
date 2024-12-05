package org.firstinspires.ftc.teamcode.CompOpModes.Autonomous.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shoddy.ShoddyPositions;
import org.firstinspires.ftc.teamcode.Shoddy.ShoddyRobotClass;
import org.firstinspires.ftc.teamcode.Shoddy.ShoddyRobotClassAuto;
import org.firstinspires.ftc.teamcode.Shoddy.ShoddyTeleOp;
import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import org.firstinspires.ftc.teamcode.CompOpModes.Autonomous.Positions.AutoPositionsNet;


@Config
@Autonomous(name="AutoNetZone", group="Auto")

public class AutoNetZone extends OpMode
{
    ShoddyRobotClassAuto r;
    ShoddyPositions po;
    AutoPositionsNet n = new AutoPositionsNet();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public boolean holdPos = false;

    //Subsystems here

    public AbsoluteAnalogEncoder rightV4BEnc;
    public AbsoluteAnalogEncoder leftV4BEnc;
    public AbsoluteAnalogEncoder rightSwivelEnc;

    //Other Shit

    double leftLinearTarget = po.LEFT_SLIDE_IN;
    double rightLinearTarget = po.RIGHT_SLIDE_IN;
    double clawTarget = po.CLAW_CLOSED;
    double wristTarget = po.WRIST_PAR;
    double botVerticalPower = 0;
    double topVerticalPower = 0;
    double intakePower = 0;

    public ElapsedTime outtakeTimer = new ElapsedTime();

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

    public enum OuttakeState {
        OUTTAKE_START,
        OUTTAKE_EXTEND,
        OUTTAKE_OPEN,
        OUTTAKE_CLOSE,
        OUTTAKE_RETRACT
    };

    public OuttakeState outtakeState = AutoNetZone.OuttakeState.OUTTAKE_START;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths()
    {

        n.scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.startPose), new Point(n.preloadPose)))
                .setLinearHeadingInterpolation(n.startPose.getHeading(), n.preloadPose.getHeading())
                .build();

        n.grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(n.preloadPose), /* Control Point */ new Point(n.pickup1ControlPose), new Point(n.pickup1Pose)))
                .setLinearHeadingInterpolation(n.preloadPose.getHeading(), n.pickup1Pose.getHeading())
                .build();

        n.scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.pickup1Pose), new Point(n.scorePose)))
                .setLinearHeadingInterpolation(n.pickup1Pose.getHeading(), n.scorePose.getHeading())
                .build();

        n.grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.scorePose), new Point(n.pickup2Pose)))
                .setLinearHeadingInterpolation(n.scorePose.getHeading(), n.pickup2Pose.getHeading())
                .build();

        n.scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.pickup2Pose), new Point(n.scorePose)))
                .setLinearHeadingInterpolation(n.pickup2Pose.getHeading(), n.scorePose.getHeading())
                .build();

        n.grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.scorePose), new Point(n.pickup3Pose)))
                .setLinearHeadingInterpolation(n.scorePose.getHeading(), n.pickup3Pose.getHeading())
                .build();

        n.scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.pickup3Pose), new Point(n.scorePose)))
                .setLinearHeadingInterpolation(n.pickup3Pose.getHeading(), n.scorePose.getHeading())
                .build();

        n.park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.scorePose), new Point(n.parkPose)))
                .setLinearHeadingInterpolation(n.scorePose.getHeading(), n.parkPose.getHeading())
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate()
    {
        switch (pathState)
        {
            case 0:
                follower.followPath(n.scorePreload);
                setPathState(1);
                break;
            case 1:
                if(follower.getPose().getX() > (n.preloadPose.getX() - 1) && follower.getPose().getY() > (n.preloadPose.getY() - 1)) {
                    /* Score Preload */

                    //claw.scoringClaw();
                    //claw.openClaw();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(n.grabPickup1, /* holdEnd = */ holdPos);
                    setPathState(2);
                }
                break;
            case 2:
                if(follower.getPose().getX() > (n.pickup1Pose.getX() - 1) && follower.getPose().getY() > (n.pickup1Pose.getY() - 1)) {

                    //Grab Sample 1

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(n.scorePickup1, /* holdEnd = */ holdPos);
                    setPathState(3);
                }
                break;
            case 3:
                if(follower.getPose().getX() > (n.scorePose.getX() - 1) && follower.getPose().getY() > (n.scorePose.getY() - 1)) {

                    //Score Sample

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(n.grabPickup2, /* holdEnd = */ holdPos);
                    setPathState(4);
                }
                break;
            case 4:
                if(follower.getPose().getX() > (n.pickup2Pose.getX() - 1) && follower.getPose().getY() > (n.pickup2Pose.getY() - 1)) {

                    //Grab Sample 2

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(n.scorePickup2, /* holdEnd = */ holdPos);
                    setPathState(5);
                }
                break;
            case 5:
                if(follower.getPose().getX() > (n.scorePose.getX() - 1) && follower.getPose().getY() > (n.scorePose.getY() - 1)) {

                    //Score Sample

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(n.grabPickup3, /* holdEnd = */ holdPos);
                    setPathState(6);
                }
                break;
            case 6:
                if(follower.getPose().getX() > (n.pickup3Pose.getX() - 1) && follower.getPose().getY() > (n.pickup3Pose.getY() - 1)) {

                    //Grab Sample 3

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(n.scorePickup3, /* holdEnd = */ holdPos);
                    setPathState(7);
                }
                break;
            case 7:
                if(follower.getPose().getX() > (n.scorePose.getX() - 1) && follower.getPose().getY() > (n.scorePose.getY() - 1)) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(n.park, /* holdEnd = */ holdPos);
                    setPathState(0);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

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

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        r = new ShoddyRobotClassAuto(this);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(n.startPose);

        buildPaths();

        //other subsystems init

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
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    //Subsystems

    public void scoreSample1()
    {
        switch (outtakeState)
        {
            case OUTTAKE_START:
                vertSlidesTarget = po.VERTICAL_UP;
                outtakeState = AutoNetZone.OuttakeState.OUTTAKE_EXTEND;
                break;
            case OUTTAKE_EXTEND:
                if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_UP) < 50) {
                    swivelTarget = po.SWIVEL_UP;
                    outtakeState = OuttakeState.OUTTAKE_OPEN;
                }
                break;
        }
    }
    public void scoreSample2()
    {

        switch (outtakeState)
        {
            case OUTTAKE_OPEN:
                outtakeTimer.reset();
                clawTarget = po.CLAW_OPEN;
                outtakeState = OuttakeState.OUTTAKE_CLOSE;
            break;
            case OUTTAKE_CLOSE:
                if (outtakeTimer.milliseconds() >= 1000)
                {
                    clawTarget = po.CLAW_CLOSED;

                    //Swivel back

                    //Slides down
                }
        }
    }

    public void setV4BPIDF(int target)
    {
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

    public void setVerticalSlidesPIDF(int target2)
    {
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

    public void setSwivelPIDF(int target3)
    {
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
package org.firstinspires.ftc.teamcode.CompOpModes.Autonomous.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shoddy.ShoddyPositions;
import org.firstinspires.ftc.teamcode.Shoddy.ShoddyRobotClass;
import org.firstinspires.ftc.teamcode.Shoddy.ShoddyRobotClassAuto;
import org.firstinspires.ftc.teamcode.Shoddy.ShoddyTeleOp;
import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import org.firstinspires.ftc.teamcode.CompOpModes.Autonomous.Positions.AutoPositionsNet;

@Config
@Autonomous(name="AutoNetZone", group="Auto")

public class AutoNetZone extends OpMode
{
    public ShoddyRobotClassAuto r;
    public ShoddyPositions po;
    public AutoPositionsNet n;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public boolean holdPos = true;

    public int xOffset;
    public int yOffset;
    public int hOffset;

    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    //Other Shit

    double leftLinearTarget;
    double rightLinearTarget;
    double clawTarget;
    double wristTarget;

    double botVerticalPower = 0;
    double topVerticalPower = 0;
    double intakePower = 0;

    public boolean usePIDFvertical = true;
    public boolean usePIDFswivel = true;
    public boolean usePIDFV4B = true;

    //First PID for V4B
    private PIDController controller;
    public static double p = 0.005, i = 0.01, d = 0.00004;
    public static double f = 0.06;
    private final double ticks_in_degree = 144.0 / 180.0;
    public int V4BTarget;
    double armPos;
    double pid, targetArmAngle, ff, currentArmAngle, V4BPower;

    //Second PID for Vertical Slides
    private PIDController controller2;
    public static double p2 = 0.006, i2 = 0.001, d2 = 0;
    public static double f2 = 0;
    private final double ticks_in_degree2 = 144.0 / 180.0;
    public int vertSlidesTarget;
    double armPos2;
    double pid2, targetArmAngle2, ff2, currentArmAngle2, verticalSlidesPower;

    //Third PID for Swivel
    private PIDController controller3;
    public static double p3 = -0.006, i3 = 0.02, d3 = 0.0002;
    public static double f3 = 0.035;
    private final double ticks_in_degree3 = 144.0 / 180.0;
    public int swivelTarget;
    double armPos3;
    double pid3, targetArmAngle3, ff3, currentArmAngle3, swivelPower;

    public int TimeVar = 100;
    public boolean loopDone = false;
    int extendTime = 400;
    int transferTime = 2000;
    int dropTime = 1000;
    int grabTime = 1800;

    int currentSample = 0;

    public PathState pathState;
    public enum PathState {
        //Shared


        //Preload
        ToPRELOAD,
        RaiseSLIDESp,
        FORWARDp,
        OpenCLAWp,
        RETREATp,
        RETRACTp,

        //Sample 1
        ToPICKUP1,
        PICKUP1,
        TRANSFER1,
        ToSCORE1a,
        RaiseSLIDES1,
        ToSCORE1b,
        OpenCLAW1,
        ToSCORE1c,
        RETRACT1,

        //Sample 2
        ToPICKUP2,
        PICKUP2,
        ToSCORE2a,

        //Sample 3
        ToPICKUP3,
        PICKUP3,
        ToSCORE3a,

        PICKUP,
        RAISE_SLIDES,
        SCORE_FORWARD,
        OPEN_CLAW,
        SCORE_RETREAT,
        SWITCH,

        TO_PICKUP_1,
        TO_PICKUP_2,
        TO_PICKUP_3,
        TO_SCORE_p,
        TO_SCORE_1,
        TO_SCORE_2,
        TO_SCORE_3,


        //Other
        PARK,
        END
    }
    public enum IntakeState {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_GRAB,
        INTAKE_RETRACT,
        INTAKE_IDLE
    };

    public enum OuttakeState {
        OUTTAKE_START,
        OUTTAKE_EXTEND,
        OUTTAKE_SWIVEL,
        OUTTAKE_IDLE
    };

    public enum TransferState {
        TRANSFER_START,
        TRANSFER_INTAKE,
        TRANSFER_CLAW,
        TRANSFER_OUTTAKE,
        TRANSFER_IDLE
    };

    public enum OuttakeState2 {
        OUTTAKE2_START,
        OUTTAKE2_RETRACT,
        OUTTAKE2_IDLE
    }
    public IntakeState intakeState = IntakeState.INTAKE_IDLE;
    public ElapsedTime intakeTimer = new ElapsedTime();
    public OuttakeState outtakeState = OuttakeState.OUTTAKE_IDLE;
    public ElapsedTime outtakeTimer = new ElapsedTime();
    public TransferState transferState = TransferState.TRANSFER_IDLE;
    public ElapsedTime transferTimer = new ElapsedTime();
    public OuttakeState2 outtakeState2 = OuttakeState2.OUTTAKE2_IDLE;
    public ElapsedTime outtakeTimer2 = new ElapsedTime();


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths()
    {

        n.scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.startPose), new Point(n.scorePosePT1)))
                .setLinearHeadingInterpolation(n.startPose.getHeading(), n.scorePosePT1.getHeading())
                .build();

        n.grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.scorePosePT1), new Point(n.pickup1Pose)))
                .setLinearHeadingInterpolation(n.scorePosePT1.getHeading(), n.pickup1Pose.getHeading())
                .build();

        n.scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.pickup1Poseb), new Point(n.scorePosePT1)))
                .setLinearHeadingInterpolation(n.pickup1Poseb.getHeading(), n.scorePosePT1.getHeading())
                .build();

        n.grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(n.scorePosePT1), new Point(n.pickup2Control), new Point(n.pickup2Pose)))
                .setLinearHeadingInterpolation(n.scorePosePT1.getHeading(), n.pickup2Pose.getHeading())
                .build();

        n.scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.pickup2Poseb), new Point(n.scorePosePT1)))
                .setLinearHeadingInterpolation(n.pickup2Poseb.getHeading(), n.scorePosePT1.getHeading())
                .build();

        n.grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.scorePosePT1), new Point(n.pickup3Pose)))
                .setLinearHeadingInterpolation(n.scorePosePT1.getHeading(), n.pickup3Pose.getHeading())
                .build();

        n.scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.pickup3Poseb), new Point(n.scorePosePT1)))
                .setLinearHeadingInterpolation(n.pickup3Poseb.getHeading(), n.scorePosePT1.getHeading())
                .build();

        n.scoreForward = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.scorePosePT1), new Point(n.scorePosePT2)))
                .setLinearHeadingInterpolation(n.scorePosePT1.getHeading(), n.scorePosePT2.getHeading())
                .build();

        n.scoreRetreat = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.scorePosePT2), new Point(n.scorePosePT1)))
                .setLinearHeadingInterpolation(n.scorePosePT2.getHeading(), n.scorePosePT1.getHeading())
                .build();

        n.park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.scorePosePT1), new Point(n.parkPose)))
                .setLinearHeadingInterpolation(n.scorePosePT1.getHeading(), n.parkPose.getHeading())
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate()
    {
        switch (pathState)
        {
            case TO_SCORE_p:
                outtakeState = OuttakeState.OUTTAKE_START;

                follower.followPath(n.scorePreload, holdPos);

                pathState = PathState.SCORE_FORWARD;
                break;
            case SCORE_FORWARD:
                if(follower.getPose().getX() > (n.scorePosePT1.getX() - 1) && follower.getPose().getY() > (n.scorePosePT1.getY() - 1) && (outtakeState == OuttakeState.OUTTAKE_IDLE))
                {
                    follower.followPath(n.scoreForward);

                    pathState = PathState.OPEN_CLAW;
                }
                break;
            case OPEN_CLAW:
                if(follower.getPose().getX() > (n.scorePosePT2.getX() - 1) && follower.getPose().getY() > (n.scorePosePT2.getY() - 1))
                {
                    pathTimer.resetTimer();
                    clawTarget = po.CLAW_OPEN;

                    pathState = PathState.SCORE_RETREAT;
                }
                break;
            case SCORE_RETREAT:
                if (pathTimer.getElapsedTime() >= 600)
                {
                    follower.followPath(n.scoreRetreat);

                    pathState = PathState.SWITCH;
                }
                break;
            case SWITCH:
                if(follower.getPose().getX() > (n.scorePosePT1.getX() - 1) && follower.getPose().getY() > (n.scorePosePT1.getY() - 1))
                {
                    if(currentSample==0)
                    {
                        currentSample = 1;
                        pathState = PathState.TO_PICKUP_1;
                    }
                    else if(currentSample==1)
                    {
                        currentSample = 2;
                        pathState = PathState.TO_PICKUP_2;
                    }
                    else if(currentSample==2)
                    {
                        currentSample = 3;
                        pathState = PathState.TO_PICKUP_3;
                    }
                    else if (currentSample==3)
                    {
                        currentSample = 0;
                        pathState = PathState.PARK;
                    }
                }
                break;
            case TO_PICKUP_1:
                    outtakeState2 = OuttakeState2.OUTTAKE2_START;

                    follower.followPath(n.grabPickup1, holdPos);

                    pathState = PathState.PICKUP;
                break;
            case TO_PICKUP_2:
                outtakeState2 = OuttakeState2.OUTTAKE2_START;

                follower.followPath(n.grabPickup2, holdPos);

                pathState = PathState.PICKUP;
                break;
            case TO_PICKUP_3:
                outtakeState2 = OuttakeState2.OUTTAKE2_START;

                follower.followPath(n.grabPickup3, holdPos);

                pathState = PathState.PICKUP;
                break;
            case PICKUP:
                if(currentSample == 1)
                {
                    if((follower.getPose().getX() > (n.pickup1Pose.getX() - 1) && follower.getPose().getY() > (n.pickup1Pose.getY() - 1)) && outtakeState2 == OuttakeState2.OUTTAKE2_IDLE)
                    {
                        intakeState = IntakeState.INTAKE_START;

                        pathState = PathState.TO_SCORE_1;
                    }
                }
                else if(currentSample == 2)
                {
                    if((follower.getPose().getX() > (n.pickup2Pose.getX() - 1) && follower.getPose().getY() > (n.pickup2Pose.getY() - 1)) && outtakeState2 == OuttakeState2.OUTTAKE2_IDLE)
                    {
                        intakeState = IntakeState.INTAKE_START;

                        pathState = PathState.TO_SCORE_2;
                    }
                }
                else if(currentSample == 3)
                {
                    if((follower.getPose().getX() > (n.pickup3Pose.getX() - 1) && follower.getPose().getY() > (n.pickup3Pose.getY() - 1)) && outtakeState2 == OuttakeState2.OUTTAKE2_IDLE)
                    {
                        intakeState = IntakeState.INTAKE_START;

                        pathState = PathState.TO_SCORE_3;
                    }
                }
                break;
            case TO_SCORE_1:
                if (intakeState == IntakeState.INTAKE_IDLE)
                {
                    follower.followPath(n.scorePickup1, holdPos);
                    transferState = TransferState.TRANSFER_START;

                    pathState = PathState.RAISE_SLIDES;
                }
            case TO_SCORE_2:
                if (intakeState == IntakeState.INTAKE_IDLE)
                {
                    follower.followPath(n.scorePickup2, holdPos);
                    transferState = TransferState.TRANSFER_START;

                    pathState = PathState.RAISE_SLIDES;
                }
            case TO_SCORE_3:
                if (intakeState == IntakeState.INTAKE_IDLE)
                {
                    follower.followPath(n.scorePickup3, holdPos);
                    transferState = TransferState.TRANSFER_START;

                    pathState = PathState.RAISE_SLIDES;
                }
                break;
            case RAISE_SLIDES:
                if(follower.getPose().getX() > (n.scorePosePT1.getX() - 1) && follower.getPose().getY() > (n.scorePosePT1.getY() - 1) && (transferState == TransferState.TRANSFER_IDLE))
                {
                    outtakeState = OuttakeState.OUTTAKE_START;

                    pathState = PathState.SCORE_FORWARD;
                }
                break;
                //Park and End
            case PARK:
                if(pathTimer.getElapsedTime() >= 2000)
                {
                    follower.followPath(n.park);

                    pathState = PathState.END;
                }
                break;
            case END:
                if(follower.getPose().getX() > (n.parkPose.getX() - 1) && follower.getPose().getY() > (n.parkPose.getY() - 1))
                {
                    //Set motors for teleop
                }


        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        poseUpdater.update();
        dashboardPoseTracker.update();

        GrabSample();
        Transfer();
        ScoreSamplePT1();
        ScoreSamplePT2();

        //Set powers
        setV4BPIDF(V4BTarget);
        setSwivelPIDF(swivelTarget);
        setVerticalSlidesPIDF(vertSlidesTarget);
        r.rightLinear.setPosition(rightLinearTarget);
        r.leftLinear.setPosition(leftLinearTarget);
        r.claw.setPosition(clawTarget);
        r.wrist.setPosition(wristTarget);
        r.intake.setPower(intakePower);

        // Feedback to Driver Hub
//        telemetry.addData("AA Intake Timer", intakeTimer.milliseconds());
//        telemetry.addData("V4B Target", targetArmAngle);
//        telemetry.addData("V4B Current Pos", currentArmAngle);
//        telemetry.addData("V4B Power", V4BPower);
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("path state", pathState);
        telemetry.addData("intake state", intakeState);
        telemetry.addData("transfer state", transferState);
        telemetry.addData("outtake1 state", outtakeState);
        telemetry.addData("outtake2 state", outtakeState2);

        //TelemetryPacket packet = new TelemetryPacket();
        //packet.fieldOverlay().setStroke("#3F51B5");
        //Drawing.drawRobot(packet.fieldOverlay(), follower.getPose());

        telemetry.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        r = new ShoddyRobotClassAuto(this);
        n = new AutoPositionsNet();
        po = new ShoddyPositions();
        poseUpdater = new PoseUpdater(hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

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

        leftLinearTarget = po.LEFT_SLIDE_IN;
        rightLinearTarget = po.RIGHT_SLIDE_IN;
        wristTarget = po.WRIST_PAR;

        double botVerticalPower = 0;
        double topVerticalPower = 0;
        double intakePower = 0;

        r.claw.setPosition(0.51);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop()
    {
        clawTarget = 0.51;
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathState = PathState.TO_SCORE_p;
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public Pose poseOffset(Pose p)
    {
        return new Pose(p.getX()+xOffset,p.getY()+yOffset,p.getHeading()+hOffset);
    }

    public void GrabSample() {
            switch (intakeState) {
                case INTAKE_START:
                        leftLinearTarget = po.LEFT_SLIDE_OUT;
                        rightLinearTarget = po.RIGHT_SLIDE_OUT;
                        intakeTimer.reset();
                        intakeState = IntakeState.INTAKE_EXTEND;
                        break;
                case INTAKE_EXTEND:
                    if (intakeTimer.milliseconds() >= extendTime) {
                        intakePower = -0.25;
                        V4BTarget = po.V4B_INTAKE_POS;
                        intakeTimer.reset();
                        intakeState = IntakeState.INTAKE_GRAB;
                    }
                    break;
                case INTAKE_GRAB:
                    if (intakeTimer.milliseconds() >= grabTime+TimeVar+200) {
                        r.LB.setPower(0);
                        r.LF.setPower(0);
                        r.RB.setPower(0);
                        r.RF.setPower(0);
                        intakePower = 0;
                        V4BTarget = po.V4B_REST_POS;
                        leftLinearTarget = po.LEFT_SLIDE_IN;
                        rightLinearTarget = po.RIGHT_SLIDE_IN;
                        intakeTimer.reset();
                        intakeState = IntakeState.INTAKE_RETRACT;
                    } else {
                        r.LB.setPower(0.45);
                        r.LF.setPower(0.45);
                        r.RB.setPower(0.45);
                        r.RF.setPower(0.45);
                    }
                    break;
                case INTAKE_RETRACT:
                    if (intakeTimer.milliseconds() >= extendTime) {
                        intakeState = IntakeState.INTAKE_IDLE;
                    }
                    break;
            }
    }

    public void Transfer(){
            switch (transferState) {
                case TRANSFER_START:
                        leftLinearTarget = po.LEFT_SLIDE_IN;
                        rightLinearTarget = po.RIGHT_SLIDE_IN;
                        V4BTarget = po.V4B_TRANSFER_POS;
                        transferState = TransferState.TRANSFER_INTAKE;
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
                        transferState = TransferState.TRANSFER_IDLE;
                    }
                    break;
            }
    }

    //ScoreSample
    public void ScoreSamplePT1() {
            switch (outtakeState) {
                case OUTTAKE_START:
                    vertSlidesTarget = po.VERTICAL_UP;
                    swivelTarget = po.SWIVEL_UP;
                    wristTarget = po.WRIST_PERP;
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.OUTTAKE_EXTEND;
                    break;
                case OUTTAKE_EXTEND:
                        outtakeState = OuttakeState.OUTTAKE_SWIVEL;

                    break;
                case OUTTAKE_SWIVEL:
                    if (outtakeTimer.milliseconds() >= extendTime+1300) {
                        outtakeState = OuttakeState.OUTTAKE_IDLE;
                    }
                    break;
            }
    }

    public void ScoreSamplePT2() {
            switch (outtakeState2) {
                case OUTTAKE2_START:
                    clawTarget = po.CLAW_CLOSED;
                    wristTarget = po.WRIST_PAR;
                    swivelTarget = po.SWIVEL_DOWN;
                    vertSlidesTarget = po.VERTICAL_REST;
                    outtakeState2 = OuttakeState2.OUTTAKE2_RETRACT;
                    break;
                case OUTTAKE2_RETRACT:
                    if (Math.abs(r.topVertical.getCurrentPosition() - po.VERTICAL_REST) <= 50) {
                        outtakeState2 = OuttakeState2.OUTTAKE2_IDLE;
                    }
                    break;
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

        //Telemetry
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
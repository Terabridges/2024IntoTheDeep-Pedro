package org.firstinspires.ftc.teamcode.CompOpModes.Autonomous.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
    AutoPositionsNet n = new AutoPositionsNet();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public boolean holdPos = false;

    //Subsystems here

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths()
    {

        n.scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.startPose), new Point(n.startOffsetPose)))
                .setLinearHeadingInterpolation(n.startPose.getHeading(), n.startOffsetPose.getHeading())
                .addPath(new BezierLine(new Point(n.startOffsetPose), new Point(n.preloadPose)))
                .setLinearHeadingInterpolation(n.startOffsetPose.getHeading(), n.preloadPose.getHeading())
                .build();

        n.grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(n.preloadPose), /* Control Point */ new Point(n.pickup1ControlPose), new Point(n.pickup1Pose)))
                .setLinearHeadingInterpolation(n.preloadPose.getHeading(), n.pickup1Pose.getHeading())
                .build();

        n.scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.pickup1Pose), new Point(n.scorePose)))
                .setLinearHeadingInterpolation(n.pickup1Pose.getHeading(), n.scorePose.getHeading())
                .build();

        /*
        n.grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.), new Point(n.)))
                .setLinearHeadingInterpolation(n..getHeading(), n..getHeading())
                .build();

        n.scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.), new Point(n.)))
                .setLinearHeadingInterpolation(n..getHeading(), n..getHeading())
                .build();

        n.grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.), new Point(n.)))
                .setLinearHeadingInterpolation(n..getHeading(), n..getHeading())
                .build();

        n.scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.), new Point(n.)))
                .setLinearHeadingInterpolation(n..getHeading(), n..getHeading())
                .build();

        n.park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(n.), new Point(n.)))
                .setLinearHeadingInterpolation(n..getHeading(), n..getHeading())
                .build();

        */
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

                    //Grab Sample

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(n.scorePickup1, /* holdEnd = */ holdPos);
                    setPathState(3);
                }
                break;
            case 3:
                //if(follower.getPose().getX() > (n.scorePose.getX() - 1) && follower.getPose().getY() > (n.scorePose.getY() - 1)) {

                    //Score Sample

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //follower.followPath(n.grabPickup1, /* holdEnd = */ holdPos);
                    //setPathState(4);
                //}
                break;
            //case 4:
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
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(n.startPose);

        buildPaths();

        //other subsystems init
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
}
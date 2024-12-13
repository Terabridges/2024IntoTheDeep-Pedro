package org.firstinspires.ftc.teamcode.CompOpModes.Autonomous.Positions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Config
public class AutoPositionsNet
{
    public double pedroSpeed = 1;

    public Pose startPose = new Pose(8.5, 87.5, Math.toRadians(180));

    public Pose scorePosePT1 = new Pose(22, 118.5, Math.toRadians(-45));

    public Pose scorePosePT2 = new Pose(21, 123.5, Math.toRadians(-45));

    /* Lowest (First) Sample from the Spike Mark */
    public Pose pickup1Pose = new Pose(30.8, 86.75, Math.toRadians(57.1));
    public Pose pickup1Poseb = new Pose(35.9, 99.5, Math.toRadians(57.1)); //After intake forward, estimated pose

    /* Middle (Second) Sample from the Spike Mark */
    public Pose pickup2Pose = new Pose(30.9, 96.75, Math.toRadians(53.2));
    //public Pose pickup2Control = new Pose(32, 118, Math.toRadians(78)); //Curve control pose
    public Pose pickup2Poseb = new Pose(36.4, 109.5, Math.toRadians(55)); //After intake forward, estimated pose

    /* Highest (Third) Sample from the Spike Mark */
    public Pose pickup3Pose = new Pose(46.25, 106.5, Math.toRadians(80));
    public Pose pickup3Poseb = new Pose(46.25, 113.8, Math.toRadians(80)); //After intake forward, estimated pose

    /* Park Pose for our robot, after we do all of the scoring. */
    public Pose parkPose = new Pose(20, 20, Math.toRadians(180));

    /** These are our Paths and PathChains that we will define in buildPaths() */
    public PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, intake1, intake2, intake3, scorePickup1, scorePickup2, scorePickup3, scoreForward, scoreRetreat, park;
}

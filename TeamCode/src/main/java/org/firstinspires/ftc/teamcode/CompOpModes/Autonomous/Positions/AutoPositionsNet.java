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
    
    /*
    |                                                                  ^
    |              < +y                                          0 deg |
    |                                <-- 90 (really 80) deg
    |          ^                                               180 deg |
    net       +x                                                       v
    ___________________
     */
    public double Off1X = 0.5;
    public double Off2X = 4.8;
    public double Off3X = 5.75;
    public double Off1Y = 0;
    public double Off2Y = 1;
    public double Off3Y = 2;

    public double globalX = -0.5;
    public double globalY = 0;

    public Pose startPose = new Pose(8.5+globalX, 87.5+globalY, Math.toRadians(180));

    public Pose scorePosePT1 = new Pose(26, 119.5, Math.toRadians(-45));

    public Pose scorePosePT2 = new Pose(25, 125, Math.toRadians(-45));

    /* Lowest (First) Sample from the Spike Mark */
    public Pose pickup1Pose = new Pose(30.8+Off1X, 86.75+Off1Y, Math.toRadians(56));
    public Pose pickup1Poseb = new Pose(35.9+Off1X, 99.5+Off1Y, Math.toRadians(56)); //After intake forward, estimated pose

    /* Middle (Second) Sample from the Spike Mark */
    public Pose pickup2Pose = new Pose(30.9+Off2X, 96.75+Off2Y, Math.toRadians(55.5));
    //public Pose pickup2Control = new Pose(32, 118, Math.toRadians(78)); //Curve control pose
    public Pose pickup2Poseb = new Pose(36.4+Off2X, 109.5+Off2Y, Math.toRadians(55.5)); //After intake forward, estimated pose

    /* Highest (Third) Sample from the Spike Mark */
    public Pose pickup3Pose = new Pose(46.25+Off3X, 106.5+Off3Y, Math.toRadians(80));
    public Pose pickup3Poseb = new Pose(46.25+Off3X, 113.8+Off3Y, Math.toRadians(80)); //After intake forward, estimated pose

    /* Park Pose for our robot, after we do all of the scoring. */
    public Pose parkPose = new Pose(20, 20, Math.toRadians(180));

    /** These are our Paths and PathChains that we will define in buildPaths() */
    public PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, intake1, intake2, intake3, scorePickup1, scorePickup2, scorePickup3, scoreForward, scoreRetreat, park;
}

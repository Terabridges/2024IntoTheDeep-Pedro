package org.firstinspires.ftc.teamcode.CompOpModes.Autonomous.Positions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Config
public class AutoPositionsNet
{
    public Pose startPose = new Pose(8.5, 87.5, Math.toRadians(180));

    /* Move Robot away from the wall to not hit while turning*/
    //public final Pose startOffsetPose = new Pose(12, 87.5, Math.toRadians(0));

    /* Place preload on rung */
    public Pose preloadPose = new Pose(30, 80.5, Math.toRadians(180));

    public Pose scorePosePT1 = new Pose(14, 120, Math.toRadians(-36));

    public Pose scorePosePT2 = new Pose(16, 120, Math.toRadians(-36));

    /* Lowest (First) Sample from the Spike Mark */
    public Pose pickup1Pose = new Pose(35, 98, Math.toRadians(65));
    //Control pose for curve
    //public static Pose pickup1ControlPose = new Pose(30, 110, Math.toRadians(87));

    /* Middle (Second) Sample from the Spike Mark */
    public Pose pickup2Pose = new Pose(45.75, 102, Math.toRadians(90));

    /* Highest (Third) Sample from the Spike Mark */
    public Pose pickup3Pose = new Pose(45.75, 112, Math.toRadians(90));

    /* Park Pose for our robot, after we do all of the scoring. */
    public Pose parkPose = new Pose(20, 124, Math.toRadians(135));

    /** These are our Paths and PathChains that we will define in buildPaths() */
    public PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, scorePickupPT1, scorePickupPT2, park;
}

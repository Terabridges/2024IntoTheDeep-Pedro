package org.firstinspires.ftc.teamcode.CompOpModes.Autonomous.Positions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Config
public class AutoPositionsNet
{
    public final Pose startPose = new Pose(8.5, 87.5, Math.toRadians(0));

    /* Move Robot away from the wall to not hit while turning*/
    public final Pose startOffsetPose = new Pose(12, 87.5, Math.toRadians(0));

    /* Place preload on rung */
    public final Pose preloadPose = new Pose(30, 80.5, Math.toRadians(180));

    public final Pose scorePose = new Pose(22, 122, Math.toRadians(-45));

    /* Lowest (First) Sample from the Spike Mark */
    public final Pose pickup1Pose = new Pose(45.75, 102, Math.toRadians(87));
    //Control pose for curve
    public final Pose pickup1ControlPose = new Pose(30, 110, Math.toRadians(87));

    /* Middle (Second) Sample from the Spike Mark */
    public final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); //fix this

    /* Highest (Third) Sample from the Spike Mark */
    public final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); //fix this

    /* Park Pose for our robot, after we do all of the scoring. */
    public final Pose parkPose = new Pose(12, 132, Math.toRadians(135));

    /** These are our Paths and PathChains that we will define in buildPaths() */
    public PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park;
}

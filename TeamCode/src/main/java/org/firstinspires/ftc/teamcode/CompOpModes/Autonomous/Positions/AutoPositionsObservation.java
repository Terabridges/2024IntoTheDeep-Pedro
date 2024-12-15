package org.firstinspires.ftc.teamcode.CompOpModes.Autonomous.Positions;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class AutoPositionsObservation
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

    public final Pose startPose = new Pose(8.5, 56.5, Math.toRadians(0));

    public Pose parkPose = new Pose(11, 21, Math.toRadians(0));

    public PathChain get1, get2, get3, park;
}

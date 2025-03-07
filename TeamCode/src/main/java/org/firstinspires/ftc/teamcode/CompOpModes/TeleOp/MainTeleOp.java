package org.firstinspires.ftc.teamcode.CompOpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utility.Toggles;
import org.firstinspires.ftc.teamcode.Utility.Positions;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.HashMap;

@Disabled
@TeleOp(name="TeleOp", group="TeleOp")
public class MainTeleOp extends LinearOpMode {

    public HashMap<String, String> gamepadMap = null;
    private ElapsedTime runtime = new ElapsedTime();
    public Pose startPose;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot r = new Robot(hardwareMap, telemetry, gamepad1, startPose, false);
        Toggles t = new Toggles(this);
        Positions po = new Positions();


        r.toInit();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            t.copyGamepad();

            r.update();
            telemetry.update();
        }
    }
}

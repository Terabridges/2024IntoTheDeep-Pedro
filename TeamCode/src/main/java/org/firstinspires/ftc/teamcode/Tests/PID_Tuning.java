package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.Shoddy.ShoddyPositions;
import org.firstinspires.ftc.teamcode.Shoddy.ShoddyRobotClass;
import org.firstinspires.ftc.teamcode.Shoddy.ShoddyToggles;
import org.firstinspires.ftc.teamcode.Utility.AbsoluteAnalogEncoder;

@Config
@TeleOp(name="PID_Tuning", group="TeleOp")
public class PID_Tuning extends LinearOpMode {

    ShoddyRobotClass r = new ShoddyRobotClass(this);
    ShoddyToggles t = new ShoddyToggles(this);
    ShoddyPositions po = new ShoddyPositions();
    private ElapsedTime runtime = new ElapsedTime();

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
    public static double p3 = -0.005, i3 = 0.01, d3 = 0.0002;
    public static double f3 = 0.035;
    private final double ticks_in_degree3 = 144.0 / 180.0;
    public static int swivelTarget;
    double armPos3;
    double pid3, targetArmAngle3, ff3, currentArmAngle3, swivelPower;

    //Fourth PID for Linear Slides
    private PIDController controller4;
    public static double p4 = 0, i4 = 0, d4 = 0;
    public static double f4 = 0;
    private final double ticks_in_degree4 = 144.0 / 180.0;
    public static int linearSlidesTarget;
    double armPos4;
    double pid4, targetArmAngle4, ff4, currentArmAngle4, linearSlidesPower;

    @Override
    public void runOpMode() {

        r.wheelSetUp();
        r.servoSetUp();
        r.motorSetUp();
        r.analogSetUp();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);
        controller3 = new PIDController(p3, i3, d3);
        controller4 = new PIDController(p4, i4, d4);

        waitForStart();
        runtime.reset();

        r.topVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()){

            ///////////////////////////////////////////////////////////////

            //setV4BPIDF(V4BTarget);
            //setVerticalSlidesPIDF(vertSlidesTarget);
            //setSwivelPIDF(swivelTarget);
            setLinearPIDF(linearSlidesTarget);

            r.leftLinear.setPower(gamepad1.left_stick_y);
            r.rightLinear.setPower(gamepad1.right_stick_y);

            /////////////////////////////////////////////////////////////////

            telemetry.addData("Linear Slides Target", linearSlidesTarget);
            telemetry.addData("Linear Slides Pos", r.rightLinearEnc.getCurrentPosition());

            telemetry.addData("Swivel Target", swivelTarget);
            telemetry.addData("Swivel Pos", armPos3);

            telemetry.addData("V4B Target", V4BTarget);
            telemetry.addData("V4B Pos", armPos);

            telemetry.addData("Vertical Slides Target", vertSlidesTarget);
            telemetry.addData("Vertical Slides Pos", armPos2);

            telemetry.addData("Linear Slides Power", r.rightLinear.getPower());

            telemetry.update();
        }
    }

    private void setV4BPIDF(int target) {
        controller.setPID(p, i, d);
        armPos = r.rightV4BEnc.getCurrentPosition();
        pid = controller.calculate(armPos, target);
        targetArmAngle = target;
        ff = (Math.sin(Math.toRadians(targetArmAngle))) * f;
        currentArmAngle = Math.toRadians((armPos) / ticks_in_degree);

        V4BPower = pid + ff;

        r.leftArm.setPower(V4BPower);
        r.leftArm.setPower(V4BPower);
    }

    private void setVerticalSlidesPIDF(int target2) {
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

    private void setSwivelPIDF(int target3) {
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

    public void setLinearPIDF(int target4) {
        controller4.setPID(p4, i4, d4);
        armPos4 = r.rightLinearEnc.getCurrentPosition();
        pid4 = controller4.calculate(armPos4, target4);
        targetArmAngle4 = target4;
        ff4 = (Math.cos(Math.toRadians(targetArmAngle4))) * f4;
        currentArmAngle4 = Math.toRadians((armPos4) / ticks_in_degree4);

        linearSlidesPower = pid4 + ff4;

        r.leftLinear.setPower(linearSlidesPower);
        r.rightLinear.setPower(linearSlidesPower);
    }

}

package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shoddy.ShoddyRobotClass;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utility.Toggles;

@Config
@TeleOp(name="ServoTest", group="Test")
public class ServoTest extends LinearOpMode {

    ShoddyRobotClass robot;

    private ElapsedTime runtime = new ElapsedTime();
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    public static double positionLeft = 0.85;
    public static double positionRight = 0.15;
    public double leftTarget = positionLeft;
    public double rightTarget = positionRight;

    @Override
    public void runOpMode(){
        robot = new ShoddyRobotClass(this);

        robot.servoSetUp();
        robot.analogSetUp();
        robot.motorSetUp();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);


            if (currentGamepad1.right_bumper && previousGamepad1.right_bumper) {
                leftTarget = positionLeft;
                rightTarget = positionRight;
            }

            robot.leftLinear.setPosition(leftTarget);
            robot.rightLinear.setPosition(rightTarget);

            telemetry.addData("Left Slide Position", robot.leftLinear.getPosition());
            telemetry.addData("Left Slide Target", positionLeft);
            telemetry.addData("Right Slide Position", robot.rightLinear.getPosition());
            telemetry.addData("Right Slide Target", positionRight);
            telemetry.update();
        }
    }
}

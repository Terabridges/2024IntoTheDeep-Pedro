package org.firstinspires.ftc.teamcode.CompOpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotBCode.RobotClassB;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utility.Positions;
import org.firstinspires.ftc.teamcode.Utility.Toggles;

import java.util.HashMap;

@Config
@TeleOp(name="LocalizerTelemetry", group="TeleOp")
public class LocalizersTelemetry extends LinearOpMode {

    public HashMap<String, String> gamepadMap = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotClassB robot = new RobotClassB(this);
        Toggles toggles = new Toggles(this);
        Positions po = new Positions();


        robot.wheelSetUpB();
        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            toggles.copyGamepad();

            //Robot Drive (Left Stick and Right Stick X)
            {
                double max;
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;
                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));
                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                leftFrontPower *= po.SPEED;
                rightFrontPower *= po.SPEED;
                leftBackPower *= po.SPEED;
                leftFrontPower *= po.SPEED;

                // Send calculated power to wheels
                robot.driveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }


            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.RobotClass;

@TeleOp(name="TeleopB", group="Main")
public class MainTeleOpB extends LinearOpMode
{
    RobotClass robot = new RobotClass(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.wheelSetUp();
        robot.servoSetUpB();
        robot.motorSetUpB();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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

                // Send calculated power to wheels
                robot.driveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }

            //Vertical Slide (Right Stick Y)
            {}

            //Claw Toggles (Bumpers)
            {}

            //Intake Spin in / out (Triggers)
            {}

            //Auto Intake 100% (A)
            {}

            //Auto Intake 50% (Y)
            {}

            //Auto Transfer (B)
            {}

            //Slow Mode Toggle (X)
            {}

        }
    }
    // Methods
}


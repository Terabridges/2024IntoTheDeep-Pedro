package org.firstinspires.ftc.teamcode.CompOpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.RobotClass;
import org.firstinspires.ftc.teamcode.Utility.Toggles;
import org.firstinspires.ftc.teamcode.Utility.Positions;

@TeleOp(name="TeleOpA", group="TeleOp")
public class MainTeleOp extends LinearOpMode {
    RobotClass robot = new RobotClass(this);
    Toggles toggles = new Toggles(this);
    Positions positions = new Positions();
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {

        robot.wheelSetUp();
        robot.servoSetUp();
        robot.motorSetUp();
        robot.analogSetUp();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
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

                leftFrontPower *= positions.speed;
                rightFrontPower *= positions.speed;
                leftBackPower *= positions.speed;
                leftFrontPower *= positions.speed;

                // Send calculated power to wheels
                robot.driveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }

            //Vertical Slide (Right Stick Y)
            {
                if (gamepad1.right_stick_y != 0){
                    //figure out PID controller
                    robot.topVertical.setPower(gamepad1.right_stick_y);
                }
            }

            //Claw Toggles (Bumpers)
            {
                toggles.toggle("right_bumper");
                if (toggles.rBumpToggle){
                    robot.rightLinear.setPosition(positions.rightSlideOut);
                    robot.leftLinear.setPosition(positions.leftSlideOut);
                } else {
                    robot.rightLinear.setPosition(positions.rightSlideIn);
                    robot.leftLinear.setPosition(positions.leftSlideIn);
                }
            }

            //Intake Spin in / out (Triggers)
            {
                //Right trigger spin in, left trigger spin out
                if (gamepad1.right_trigger > 0) {
                    robot.intake.setPower(-1 * (gamepad1.right_trigger) / 2);
                } else if (gamepad1.left_trigger > 0){
                    robot.intake.setPower(1 * (gamepad1.left_trigger) / 2);
                }
            }

            //Auto Intake 100% (A)
            {}

            //Auto Intake 50% (Y)
            {}

            //Auto Transfer (B)
            {}

            //Slow Mode Toggle (X)
            {
                toggles.toggle("x");
                if (toggles.slowModeToggle){
                    positions.speed *= positions.slowMultiplier;
                } else {
                    positions.speed = positions.MaxSpeed;
                }
            }

        }
    }
    // Methods
}

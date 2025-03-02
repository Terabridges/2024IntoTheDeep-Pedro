package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.RobotClass;
import org.firstinspires.ftc.teamcode.Utility.Toggles;

@TeleOp(name="TeleOpA", group="Main")
public class MainTeleOpA extends LinearOpMode {
    RobotClass robot = new RobotClass(this);
    Toggles toggles = new Toggles(this);
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {

        robot.wheelSetUp();
        robot.servoSetUpA();
        robot.motorSetUpA();

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
                // Send calculated power to wheels
                if (toggles.slowModeToggle) {
                    robot.driveRobot(((leftFrontPower)/2), ((rightFrontPower)/2), ((leftBackPower)/2), ((rightBackPower)/2));
                } else {
                    robot.driveRobot(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
                }
            }

            //Vertical Slide (Right Stick Y)
            {

            }

            //Claw Toggles (Bumpers)
            {
                toggles.toggle("right_bumper");
                if (toggles.rBumpToggle){
                    robot.leftSlideServo.setPosition(0);
                    robot.rightSlideServo.setPosition(1);
                } else {
                    robot.leftSlideServo.setPosition(0.15);
                    robot.rightSlideServo.setPosition(0.85);
                }
            }

            //Intake Spin in / out (Triggers)
            {
                //Just in right now
                robot.intakeServo.setPower(-1*(gamepad1.right_trigger)/2);
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
            }

        }
    }
    // Methods
}

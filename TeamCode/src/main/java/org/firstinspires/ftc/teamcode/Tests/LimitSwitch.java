package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LimitSwitch extends LinearOpMode {
    // Define variables for our touch sensor and motor
    TouchSensor limit;

    @Override
    public void runOpMode() {
        // Get the touch sensor and motor from hardwareMap
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limit = hardwareMap.get(TouchSensor.class, "Limit");

        // Wait for the play button to be pressed
        waitForStart();

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            // If the Magnetic Limit Swtch is pressed, stop the motor
            if (limit.isPressed()) {
                telemetry.addData("Button...", "Pressed");
            } else { // Otherwise, run the motor
                telemetry.addData("Button...", "NOT");
            }

            telemetry.update();
        }
    }
}

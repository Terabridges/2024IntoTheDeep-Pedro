package org.firstinspires.ftc.teamcode.Shoddy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ShoddyToggles {
    private LinearOpMode myOpMode = null;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public boolean lBumpToggle = false;
    public boolean rBumpToggle = false;
    public boolean aToggle = false;
    public boolean yToggle = false;
    public boolean bToggle = false;
    public boolean xToggle = false;
    public boolean dpRightToggle = false;
    public boolean dpLeftToggle = false;
    public boolean dpUpToggle = false;
    public boolean dpDownToggle = false;
    public boolean lStickToggle = false;


    public ShoddyToggles(LinearOpMode opmode) {
        myOpMode = opmode;
    }


    public void copyGamepad(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(myOpMode.gamepad1);
    }

    public boolean toggle(String button) {
        if (button.equals("right_bumper")) {
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                rBumpToggle = !rBumpToggle;
                return true;
            }
        } else if (button.equals("left_bumper")) {
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                lBumpToggle = !lBumpToggle;
                return true;
            }
        } else if (button.equals("x")) {
            if (currentGamepad1.x && !previousGamepad1.x) {
                xToggle = !xToggle;
                return true;
            }
        } else if (button.equals("a")) {
            if (currentGamepad1.a && !previousGamepad1.a) {
                aToggle = !aToggle;
                return true;
            }
        } else if (button.equals("y")) {
            if (currentGamepad1.y && !previousGamepad1.y) {
                yToggle = !yToggle;
                return true;
            }
        } else if (button.equals("b")) {
            if (currentGamepad1.b && !previousGamepad1.b) {
                bToggle = !bToggle;
                return true;
            }
        } else if (button.equals("dpad_right")) {
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                dpRightToggle = !dpRightToggle;
                return true;
            }
        } else if (button.equals("dpad_left")) {
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                dpLeftToggle = !dpLeftToggle;
                return true;
            }
        } else if (button.equals("dpad_up")) {
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                dpUpToggle = !dpUpToggle;
                return true;
            }
        } else if (button.equals("left_stick_button")) {
            if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
                lStickToggle = !lStickToggle;
                return true;
            }
        } else if (button.equals("dpad_down")) {
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                dpDownToggle = !dpDownToggle;
                return true;
            }
        }
        return false;
    }
}

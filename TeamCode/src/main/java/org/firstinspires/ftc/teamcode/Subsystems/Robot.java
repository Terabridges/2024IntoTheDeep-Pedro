package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Robot {

    //Objects
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gp1;
    public static VoltageSensor voltageSensor;

    //Subsystems
    public IntakeSystem intakeSystem;
    public OuttakeSystem outtakeSystem;
    public TransferSystem transferSystem;
    public Drivebase drivebase;

    //Subsystem List
    public List<Subsystem> subsystems;

    //Constructors
    public Robot(HardwareMap map, Telemetry t, Gamepad gp1, Pose start, boolean resetSlidesEncoder){
        hardwareMap = map;
        telemetry = t;

        drivebase = new Drivebase(hardwareMap);

        outtakeSystem = new OuttakeSystem(hardwareMap);
        intakeSystem = new IntakeSystem(hardwareMap);
        transferSystem = new TransferSystem(hardwareMap);

        subsystems = new ArrayList<>(Arrays.asList(outtakeSystem, intakeSystem, transferSystem, drivebase));

        this.gp1 = gp1;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


    //Methods
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }


    //Interface Methods
    public void update() {
        for (Subsystem s : subsystems) {
            s.update();
        }
    }


    public void toInit() {
        for (Subsystem s : subsystems) {
            s.toInit();
        }
    }

}

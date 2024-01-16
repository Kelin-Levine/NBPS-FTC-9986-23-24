package org.firstinspires.ftc.teamcode.automode2launcher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumAutoMode2Launcher;

@Autonomous(name="Gary Mecanum Autonomous Mode 2 (Testing)", group="Robot")
public class Testing extends MecanumAutoMode2Launcher {
    @Override
    public void runOpMode() {
        selectedRoutine = Routine.TESTING;
        super.runOpMode();
    }
}

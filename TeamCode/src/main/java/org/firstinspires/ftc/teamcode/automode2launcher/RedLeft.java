package org.firstinspires.ftc.teamcode.automode2launcher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumAutoMode2Launcher;

@Autonomous(name="Gary Mecanum Autonomous Mode 2 (Red-Left)", group="Robot")
public class RedLeft extends MecanumAutoMode2Launcher {
    @Override
    public void runOpMode() {
        isBlueSide = false;
        isLongDistance = true;
        selectedRoutine = Routine.NORMAL;
        isDoingAlternateRoute = false;
        super.runOpMode();
    }
}

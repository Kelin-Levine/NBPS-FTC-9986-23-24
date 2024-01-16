package org.firstinspires.ftc.teamcode.automode2launcher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumAutoMode2Launcher;

@Autonomous(name="Gary Mecanum Autonomous Mode 2 (Blue-Left)", group="Robot")
public class BlueLeft extends MecanumAutoMode2Launcher {
    @Override
    public void runOpMode() {
        isBlueSide = true;
        isLongDistance = false;
        selectedRoutine = Routine.NORMAL;
        isDoingAlternateRoute = false;
        super.runOpMode();
    }
}

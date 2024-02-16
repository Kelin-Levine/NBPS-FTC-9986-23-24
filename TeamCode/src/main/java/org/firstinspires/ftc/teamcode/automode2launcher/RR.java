package org.firstinspires.ftc.teamcode.automode2launcher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MecanumAutoMode2Launcher;

@Disabled
@Autonomous(name="Gary Mecanum Autonomous Mode 2 (Red-Right)", group="Robot")
public class RR extends MecanumAutoMode2Launcher {
    @Override
    public void runOpMode() {
        isBlueSide = false;
        isLongDistance = false;
        selectedRoutine = Routine.NORMAL;
        isDoingAlternateRoute = false;
        super.runOpMode();
    }
}

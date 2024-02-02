package org.firstinspires.ftc.teamcode.automode3launcher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumAutoMode3;

@Autonomous(name="Gary Mecanum Autonomous Mode 3 (Red-Left)", group="Robot")
public class RedLeft extends MecanumAutoMode3 {
    @Override
    public void runOpMode() {
        isBlueSide = false;
        isLongDistance = true;
        selectedRoutine = Routine.NORMAL;
        isDoingAlternateRoute = false;
        super.runOpMode();
    }
}

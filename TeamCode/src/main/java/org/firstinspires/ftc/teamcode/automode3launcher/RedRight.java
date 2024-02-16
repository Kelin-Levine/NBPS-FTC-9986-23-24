package org.firstinspires.ftc.teamcode.automode3launcher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MecanumAutoMode3;

@Disabled
@Autonomous(name="Gary Mecanum Autonomous Mode 3 (Red-Right)", group="Robot")
public class RedRight extends MecanumAutoMode3 {
    @Override
    public void runOpMode() {
        isBlueSide = false;
        isLongDistance = false;
        selectedRoutine = Routine.NORMAL;
        isDoingAlternateRoute = false;
        super.runOpMode();
    }
}

package org.firstinspires.ftc.teamcode.automode3launcher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MecanumAutoMode3;

@Disabled
@Autonomous(name="Gary Mecanum Autonomous Mode 3 (Blue-Left)", group="Robot")
public class BlueLeft extends MecanumAutoMode3 {
    @Override
    public void runOpMode() {
        isBlueSide = true;
        isLongDistance = false;
        selectedRoutine = Routine.NORMAL;
        isDoingAlternateRoute = false;
        super.runOpMode();
    }
}

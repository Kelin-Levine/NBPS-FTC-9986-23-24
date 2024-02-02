package org.firstinspires.ftc.teamcode.automode3launcher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumAutoMode3;

@Autonomous(name="Gary Mecanum Autonomous Mode 3 (Testing)", group="Robot")
public class Testing extends MecanumAutoMode3 {
    @Override
    public void runOpMode() {
        selectedRoutine = Routine.TESTING;
        super.runOpMode();
    }
}

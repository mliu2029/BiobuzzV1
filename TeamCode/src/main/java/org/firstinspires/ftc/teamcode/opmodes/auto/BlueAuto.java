package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.BaseModularAuto;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Configurable
@NextAutonomous(name = "Blue Auto", group = "Auto")
public class BlueAuto extends BaseModularAuto {
    public BlueAuto(Robot robot) {
        super(robot, Alliance.BLUE);
    }
}

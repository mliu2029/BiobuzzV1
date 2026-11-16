package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.robot.opmode.NextAutonomous;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.auto.modular.BaseModularAuto;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Configurable
@NextAutonomous(name = "Red Auto", group = "Auto")
public class RedAuto extends BaseModularAuto {
    public RedAuto(Robot robot) {
        super(robot, Alliance.RED);
    }
}

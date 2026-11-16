import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.commands.Commands.*;


@Autonomous
public class ElennaAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(56.2, 8.6, 90);
    private final Pose gardenPose = p.of(9.9, 8.2, -90);
    private final Pose park = p.of(10.59, 9.1, 0);
    private Path garden() {
        return line(startPose, gardenPose).linear(startPose, gardenPose);
    }

    private Path park() {
        return line(gardenPose, park).linear(gardenPose, park);
    }
    Command pause = waitMs(500);

    private Command autoRoutine() {
        return sequential(
                pause,
                follow(follower, garden()),
                // Add mechanism commands here.
                follow(follower, park())
        );
    }

    @Override

    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

}

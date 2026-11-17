package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.paths.Path;
import java.lang.reflect.Field;
import org.junit.Before;
import org.junit.Test;

public class DrivetrainPathTest {
    private final FakeFollower follower = new FakeFollower();
    private final TestDrivetrain drivetrain = new TestDrivetrain();

    @Before
    public void injectFollower() throws Exception {
        Field field = Drivetrain.class.getDeclaredField("follower");
        field.setAccessible(true);
        field.set(drivetrain, follower);
    }

    @Test
    public void successfulPathReportsCompletionWithoutStoppingHold() {
        Command command = drivetrain.followPath(null);
        command.start();
        assertEquals(Drivetrain.PathResult.RUNNING, drivetrain.getLastPathResult());
        assertFalse(command.done());
        follower.atEnd = true;
        follower.busy = false;
        assertTrue(command.done());
        command.end(EndCondition.NATURALLY);
        assertEquals(Drivetrain.PathResult.COMPLETED, drivetrain.getLastPathResult());
        assertEquals(0, drivetrain.stops);
        assertEquals(1, follower.follows);
    }

    @Test
    public void parametricEndAloneDoesNotMeanSettled() {
        Command command = drivetrain.followPath(null);
        command.start();
        follower.atEnd = true;
        assertFalse(command.done());
    }

    @Test
    public void expiredPathReportsTimeoutAndStops() throws Exception {
        Command command = drivetrain.followPath(null);
        command.start();
        expirePath();
        assertTrue(command.done());
        command.end(EndCondition.NATURALLY);
        assertEquals(Drivetrain.PathResult.TIMED_OUT, drivetrain.getLastPathResult());
        assertEquals(1, drivetrain.stops);
    }

    @Test
    public void interruptedPathCannotReportSuccess() {
        Command command = drivetrain.followPath(null);
        command.start();
        follower.atEnd = true;
        follower.busy = false;
        command.end(EndCondition.INTERRUPTED);
        assertEquals(Drivetrain.PathResult.CANCELLED, drivetrain.getLastPathResult());
        assertEquals(1, drivetrain.stops);
    }

    @Test
    public void newPathClearsPreviousTimeout() throws Exception {
        Command first = drivetrain.followPath(null);
        first.start();
        expirePath();
        first.end(EndCondition.NATURALLY);
        Command second = drivetrain.followPath(null);
        second.start();
        assertEquals(Drivetrain.PathResult.RUNNING, drivetrain.getLastPathResult());
        assertFalse(second.done());
    }

    private void expirePath() throws Exception {
        Field field = Drivetrain.class.getDeclaredField("pathStartedNanos");
        field.setAccessible(true);
        field.setLong(drivetrain, System.nanoTime() - 5_000_000_001L);
    }

    private static final class TestDrivetrain extends Drivetrain {
        int stops;

        @Override
        public void stopImmediately() {
            stops++;
        }
    }

    private static final class FakeFollower extends Follower {
        boolean atEnd;
        boolean busy = true;
        int follows;

        FakeFollower() {
            super(null, null, null);
        }

        @Override
        public void follow(Path path) {
            follows++;
        }

        @Override
        public boolean atParametricEnd() {
            return atEnd;
        }

        @Override
        public boolean isBusy() {
            return busy;
        }
    }
}

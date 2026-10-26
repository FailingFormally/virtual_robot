package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedropathing.tuning.Constants;

@Autonomous(name = "AutoPath2", group = "Autonomous")
public class autopath2 extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(62.7193, 132.8994, 90);
    private final Pose shootPreloadStart = poseFactory.of(62.7193, 132.8994, 270);
    private final Pose shootPreload = poseFactory.of(58.3715, 107.1453, 270);
    private final Pose gatherFlower2Pollen = poseFactory.of(11.7402, 41.419, 180);
    private final Pose gatherFlower2PollenControl1 = poseFactory.of(11.1941, 120.4539, 0);
    private final Pose gatherFlower2PollenControl2 = poseFactory.of(47.1899, 38.1508, 0);
    private final Pose shootFlower2Pollen = poseFactory.of(58.7905, 35.4749, 90);
    private final Pose gatherUndergoal = poseFactory.of(58.5098, 106.6634, 0);
    private final Pose gatherUndergoalSegment1Start = poseFactory.of(58.5098, 106.6634, 90);
    private final Pose gatherUndergoalSegment1End = poseFactory.of(58.5098, 106.6634, 90);
    private final Pose gatherUndergoalSegment2Start = poseFactory.of(58.5098, 106.6634, 90);
    private final Pose gatherUndergoalSegment2End = poseFactory.of(58.5098, 106.6634, 0);
    private final Pose parkStart = poseFactory.of(58.5098, 106.6634, 90);
    private final Pose park = poseFactory.of(10.8701, 120.655, 164);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shootPreload()),
                follow(follower, gatherFlower2Pollen()),
                follow(follower, shootFlower2Pollen()),
                follow(follower, gatherUndergoal()),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path shootPreload() {
        return Paths.line(shootPreloadStart, shootPreload).linear(shootPreloadStart, shootPreload);
    }

    public Path gatherFlower2Pollen() {
        return Paths.curve(shootPreload, gatherFlower2PollenControl1, gatherFlower2PollenControl2, gatherFlower2Pollen).linear(shootPreload, gatherFlower2Pollen);
    }

    public Path shootFlower2Pollen() {
        return Paths.line(gatherFlower2Pollen, shootFlower2Pollen).linear(gatherFlower2Pollen, shootFlower2Pollen);
    }

    public Path gatherUndergoal() {
        return Paths.line(shootFlower2Pollen, gatherUndergoal).heading(Interpolator.piecewise().until(0.7074, Interpolator.linear(gatherUndergoalSegment1Start, gatherUndergoalSegment1End)).until(1, Interpolator.linear(gatherUndergoalSegment2Start, gatherUndergoalSegment2End)));
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}

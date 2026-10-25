package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.162, 8.7905, 90);
    private final Pose shootPreloads = poseFactory.of(58.7668, 35.6047, 90);
    private final Pose pickupGardenPollen = poseFactory.of(7.9497, 9.4358, 270);
    private final Pose pickupGardenPollenControl1 = poseFactory.of(41.6522, 26.831, 0);
    private final Pose pickupGardenPollenControl2 = poseFactory.of(14.4637, 35.3883, 0);
    private final Pose shootGardenPollen = poseFactory.of(58.8226, 106.3547, 270);
    private final Pose shootGardenPollenControl1 = poseFactory.of(83.7151, 7.2263, 0);
    private final Pose shootGardenPollenControl2 = poseFactory.of(50.514, 100.067, 0);
    private final Pose gatherFlowerPollen = poseFactory.of(42.1425, 129.6397, 90);
    private final Pose shootFlowerPollen = poseFactory.of(58.4274, 35.6285, 90);
    private final Pose shootFlowerPollenControl1 = poseFactory.of(5.5265, 67.3953, 0);
    private final Pose shootFlowerPollenControl2 = poseFactory.of(34.1662, 13.764, 0);
    private final Pose park = poseFactory.of(11.8115, 89.3869, 135);
    private final Pose parkControl1 = poseFactory.of(22.8115, 23.7556, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shootPreloads()),
                follow(follower, pickupGardenPollen()),
                follow(follower, shootGardenPollen()),
                follow(follower, gatherFlowerPollen()),
                follow(follower, shootFlowerPollen()),
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

    public Path shootPreloads() {
        return Paths.line(start, shootPreloads).linear(start, shootPreloads);
    }

    public Path pickupGardenPollen() {
        return Paths.curve(shootPreloads, pickupGardenPollenControl1, pickupGardenPollenControl2, pickupGardenPollen).linear(shootPreloads, pickupGardenPollen);
    }

    public Path shootGardenPollen() {
        return Paths.curve(pickupGardenPollen, shootGardenPollenControl1, shootGardenPollenControl2, shootGardenPollen).linear(pickupGardenPollen, shootGardenPollen);
    }

    public Path gatherFlowerPollen() {
        return Paths.line(shootGardenPollen, gatherFlowerPollen).linear(shootGardenPollen, gatherFlowerPollen);
    }

    public Path shootFlowerPollen() {
        return Paths.curve(gatherFlowerPollen, shootFlowerPollenControl1, shootFlowerPollenControl2, shootFlowerPollen).linear(gatherFlowerPollen, shootFlowerPollen);
    }

    public Path park() {
        return Paths.curve(shootFlowerPollen, parkControl1, park).linear(shootFlowerPollen, park);
    }
}
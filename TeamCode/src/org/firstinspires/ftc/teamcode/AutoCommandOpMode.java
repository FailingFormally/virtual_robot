package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name="Auto for Back Position, Blue Team")
public class AutoCommandOpMode extends OpMode {

    private Follower follower;
    private PathChain pathToScoringPosition;

    private final Pose startPose = new Pose(0,0, Math.toRadians(0));
    private final Pose endPose = new Pose(64, 0, Math.toRadians(35));

    private Telemetry telemetryA;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        pathToScoringPosition = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(endPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();

        follower.followPath(pathToScoringPosition);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run a straight line from the back,"
                + "of the field toward the goal. So, make sure you have enough "
                + "space in front of you to run the OpMode.");
        telemetryA.update();
    }

    @Override
    public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(pathToScoringPosition, true);
        }

        follower.telemetryDebug(telemetryA);
    }
}

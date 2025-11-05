package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name="Auto for Back Position, Blue Team")
public class AutoBackBlueOpMode extends OpMode {

    enum PathState {
        START,
        START_TO_SCORE_POS,
        SCORE_PRELOAD,
        GATHER_PPG,
        MOVE_TO_SCORE_LEG2,
        END
    }

    private PathState pathState;

    private Follower follower;
    private PathChain pathStartToScoringPosition;
    private PathChain pathToGatherRow1;

    private final Pose startPose = new Pose(0,0, Math.toRadians(0));
    private final Pose scorePose = new Pose( 64, 0, Math.toRadians(35));

    private final Pose row1Intermediate = new Pose(28, 0, Math.toRadians(90));

    private final Pose row1End = new Pose(28, 32, Math.toRadians(90));

    private Telemetry telemetryA;

    private void buildPaths() {
        pathStartToScoringPosition = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        pathToGatherRow1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(row1Intermediate)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), row1Intermediate.getHeading())
                .addPath(new BezierLine(new Point(row1Intermediate), new Point(row1End)))
                .addPath(new BezierLine(new Point(row1End), new Point(row1Intermediate)))
                .setLinearHeadingInterpolation(row1End.getHeading(), row1Intermediate.getHeading())
                .addPath(new BezierLine(new Point(row1Intermediate), new Point(scorePose)))
                .setLinearHeadingInterpolation(row1Intermediate.getHeading(), scorePose.getHeading())
                .build();

    }

    @Override
    public void init() {
        pathState = PathState.START;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        buildPaths();
    }

    private void pathUpdate() {
        switch(pathState) {
            case START:
                follower.followPath(pathStartToScoringPosition);
                pathState = PathState.START_TO_SCORE_POS;
                break;
            case START_TO_SCORE_POS:
                if (follower.atParametricEnd()) {
                    follower.followPath(pathStartToScoringPosition, true);
                }
                follower.telemetryDebug(telemetryA);
                if (!follower.isBusy()) {
                    pathState = PathState.SCORE_PRELOAD;
                    resetStartTime(); // Timer for shooting
                }
                break;
            case SCORE_PRELOAD:
                telemetry.addData("Action", "Pew Pew Pew");
                telemetry.update();

                pathState = PathState.GATHER_PPG;
                follower.followPath(pathToGatherRow1);
                break;
            case GATHER_PPG:
                follower.telemetryDebug(telemetryA);
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        pathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}

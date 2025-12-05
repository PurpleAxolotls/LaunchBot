package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "backShotsBlue", group = "Examples")
public class backShots extends OpMode {

    double flyWheelTargetVelocity;
    double flyWheelRPM = 1750;
    double flywheelTPR = 28;

    DcMotor chamberMotor = null;
    CRServo rightBackChamber = null;
    CRServo upperRightChamber = null;
    CRServo specialChamber = null;


    Servo gate = null;

    DcMotor intake = null;

    DcMotorEx leftFlyWheel = null;
    DcMotorEx rightFlyWheel = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // Poses

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    double refX = 87.25;
    Pose StartPos = new Pose(refX, 9, Math.toRadians(90));

    public void Paths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(87.250, 9.000), new Pose(87.250, 12.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(75))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(87.250, 12.000), new Pose(100.000, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 36.000), new Pose(132.000, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(132.000, 36.000), new Pose(87.250, 12.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(75))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(87.250, 12.000), new Pose(135.125, 30.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(75), Math.toRadians(90))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(135.125, 30.000), new Pose(135.125, 9.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(135.125, 9.000), new Pose(87.250, 12.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(75))
                .build();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case -1:
                gate.setPosition(0);
                chamberMotor.setPower(-1);
                rightBackChamber.setPower(1);
                upperRightChamber.setPower(1);
                intake.setPower(1);
                setPathState(0);
                break;
            case 0:
                follower.followPath(Path1);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    shoot();
                }
                break;

            case 3:

                if (!follower.isBusy()) {
                    follower.followPath(Path2);
                    setPathState(4);
                }

                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Path3, 0.25, true);
                    setPathState(5);
                }

                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(Path4);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    shoot();
                }
                break;
            case 7:
                /*
                follower.followPath(Path5);
                setPathState(8);
                */
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(Path6,0.25,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(Path7);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    shoot();
                }
        }

        /*
        if (opmodeTimer.getElapsedTimeSeconds() > 26) {
            setPathState(98);
        }
         */
    }

    public void shoot() {

        gate.setPosition(1);

        if (Math.abs(leftFlyWheel.getVelocity() - targetTPS) < 25){
            specialChamber.setPower(-1);
        } else {
            specialChamber.setPower(0);
        }

        if (pathTimer.getElapsedTimeSeconds() > 8) {
            setPathState(pathState + 1);
            specialChamber.setPower(0);
            gate.setPosition(0);
        }
    }



    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    double actualRPM = 0;
    double targetTPS = 0;

    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        targetTPS = (flyWheelRPM / 60) * flywheelTPR;
        leftFlyWheel.setVelocity(targetTPS);
        rightFlyWheel.setVelocity(targetTPS);

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("lefttargetTPS",targetTPS);
        telemetry.addData("leftActualTPS",leftFlyWheel.getVelocity());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(StartPos);
        Paths(follower);

        upperRightChamber = hardwareMap.get(CRServo.class, "frontRightS");
        specialChamber = hardwareMap.get(CRServo.class, "specialChamber");
        chamberMotor = hardwareMap.get(DcMotor.class, "chamberMotor");
        rightBackChamber = hardwareMap.get(CRServo.class, "backRightChamber");

        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");

        leftFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        gate = hardwareMap.get(Servo.class, "gate");

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(-1);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}
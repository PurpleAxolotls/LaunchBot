package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.bylazar.configurables.annotations.Configurable;
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

@Autonomous(name = "RedThreeShooter", group = "Examples")
public class redThreeShooter extends OpMode {

    double flyWheelTargetVelocity;
    double flyWheelRPM = 1750;
    double flywheelTPR = 28;
    CRServo lowerLeftChamber = null;
    CRServo lowerRightChamber = null;
    CRServo upperLeftChamber = null;
    CRServo upperRightChamber = null;
    CRServo specialChamber = null;

    Servo gate = null;

    DcMotor intake = null;

    DcMotorEx leftFlyWheel = null;
    DcMotorEx rightFlyWheel = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    public PathChain Path1;
    public PathChain Path2;

    public PathChain Path3;

    public void Paths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 17.75), new Pose(88, 87))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 87), new Pose(88, 87))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88,87), new Pose(88,110))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135) , Math.toRadians(90))
                .build();
    }



    double VelTolerance = 0.05;
    public void autonomousPathUpdate(double targetTPS) {

        switch (pathState) {
            case -1:
                gate.setPosition(0);
                lowerLeftChamber.setPower(-1);
                lowerRightChamber.setPower(1);
                upperLeftChamber.setPower(-1);
                upperRightChamber.setPower(1);
                intake.setPower(-1);
                if (pathTimer.getElapsedTimeSeconds() > 14) {
                    setPathState(0);
                }
                break;
            case 0:
                follower.followPath(Path1,.75,true);
                setPathState(2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    specialChamber.setPower(-1);
                    gate.setPosition(1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 12)
                {
                    setPathState(3);
                    specialChamber.setPower(0);
                    gate.setPosition(0);
                }
                break;
            case 3:
                specialChamber.setPower(0);
                lowerLeftChamber.setPower(0);
                lowerRightChamber.setPower(0);
                upperLeftChamber.setPower(0);
                upperRightChamber.setPower(0);
                follower.followPath(Path3);
                flyWheelRPM = 0;
                setPathState(999);
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/



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
        autonomousPathUpdate(targetTPS);

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
        follower.setStartingPose(new Pose(88, 17.75, Math.toRadians(90)));
        Paths(follower);

        lowerLeftChamber = hardwareMap.get(CRServo.class, "backLeftS");
        lowerRightChamber = hardwareMap.get(CRServo.class, "backRightS");
        upperLeftChamber = hardwareMap.get(CRServo.class, "frontLeftS");
        upperRightChamber = hardwareMap.get(CRServo.class, "frontRightS");
        specialChamber = hardwareMap.get(CRServo.class, "specialChamber");

        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");

        leftFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

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
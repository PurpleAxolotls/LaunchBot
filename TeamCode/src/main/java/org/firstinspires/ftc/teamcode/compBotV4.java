/*
   Program Purpose:

   Version of Robot with complete functionality added, however with complete manual control
   over output RPM.

 */

package org.firstinspires.ftc.teamcode;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="compBotV4")
public class compBotV4 extends OpMode {

    CRServo lowerLeftChamber = null;
    CRServo lowerRightChamber = null;
    CRServo upperLeftChamber = null;
    CRServo upperRightChamber = null;
    CRServo specialChamber = null;

    DcMotor intake = null;

    DcMotor leftFront = null;
    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor rightBack = null;

    double max;
    double axial;
    double lateral;
    double yaw;
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    DcMotorEx leftFlyWheel = null;
    DcMotorEx rightFlyWheel = null;
    double flyWheelDesiredRPM = 2000;
    double flywheelTPR = 28;

    Servo gate = null;

    boolean dpad_up_pressed_previous = false;
    boolean dpad_down_pressed_previous = false;
    boolean a_pressed_previous = false;
    boolean x_pressed_previous = false;
    boolean y_pressed_previous = false;
    double fallbackRPM = 2000;
    boolean locked = false;
    AprilTagProcessor aprilTag = null;
    VisionPortal visionPortal = null;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0.5, 2.25, 8.25, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private Follower follower;

    @Override
    public void init() {

        // Chamber Servos
        lowerLeftChamber = hardwareMap.get(CRServo.class, "backLeftS");
        lowerRightChamber = hardwareMap.get(CRServo.class, "backRightS");
        upperLeftChamber = hardwareMap.get(CRServo.class, "frontLeftS");
        upperRightChamber = hardwareMap.get(CRServo.class, "frontRightS");
        specialChamber = hardwareMap.get(CRServo.class, "specialChamber");

        // Intake
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Wheels
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Fly Wheels
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");

        leftFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Gate
        gate = hardwareMap.get(Servo.class, "gate");

        follower = Constants.createFollower(hardwareMap);

        initAprilTag();

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.setStartingPose(new Pose(0,0,90));
        intake.setPower(-1);
    }

    @Override
    public void loop() {

        follower.update();

        if (gamepad2.a && !a_pressed_previous) {
            gateLogic();
        }
        a_pressed_previous = gamepad2.a;

        if (!locked) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true);
        }

        if (gamepad1.x && !x_pressed_previous) {
            locked = !locked;
            if (!locked) {
                follower.startTeleopDrive();
            } else {
                follower.holdPoint(new Pose(follower.getPose().getX(),follower.getPose().getY(),45));
            }
        }
        x_pressed_previous = gamepad1.x;

        if (gamepad1.y && !y_pressed_previous) {
            follower.setPose(telemetryFirstAprilTag());
        }
        y_pressed_previous = gamepad1.y;

        flyWheelLogic();
        chamberLogic();

        telemetry.addData("Target RPM", flyWheelDesiredRPM);
        telemetry.addData("Actual RPM Left", "%.2f", (leftFlyWheel.getVelocity() * 60) / flywheelTPR);
        telemetry.addData("Actual RPM Right", "%.2f", (rightFlyWheel.getVelocity() * 60) / flywheelTPR);
        telemetry.addData("PedroPoint", follower.getPose().toString());

        telemetry.update();

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }

    public void chamberLogic() {
        lowerLeftChamber.setPower(-1);
        lowerRightChamber.setPower(1);
        upperLeftChamber.setPower(-1);
        upperRightChamber.setPower(1);
    }

    public void flyWheelLogic() {

        if (gamepad2.left_bumper) {
            flyWheelDesiredRPM = 4000;
        } else if (gamepad2.right_bumper) {
            flyWheelDesiredRPM = 5500;
        } else {
            flyWheelDesiredRPM = fallbackRPM;
        }

        if ((gamepad2.dpad_up && !dpad_up_pressed_previous)) {
            fallbackRPM += 100;
        }
        dpad_up_pressed_previous = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !dpad_down_pressed_previous) {
            fallbackRPM -= 100;
        }
        dpad_down_pressed_previous = gamepad2.dpad_down;

        double flyWheelTargetVelocity = (flyWheelDesiredRPM / 60) * flywheelTPR;

        leftFlyWheel.setVelocity(flyWheelTargetVelocity);
        rightFlyWheel.setVelocity(flyWheelTargetVelocity);

    }

    public void gateLogic() {
        if(gate.getPosition() == 1) {
            gate.setPosition(0);
            specialChamber.setPower(0);
        } else if (gate.getPosition() == 0) {
            gate.setPosition(1);
            specialChamber.setPower(-1);
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private Pose telemetryFirstAprilTag() {

        // Get the current list of AprilTag detections.
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        // Check if any tags are detected.
        if (currentDetections.size() > 0) {
            // Get the first detection from the list.
            AprilTagDetection detection = currentDetections.get(0);
            // Check that the tag has valid metadata.
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n=====[ First Detection ID %d ]=====", detection.id));
                telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("YPR %6.2f %6.2f %6.2f  (deg)", detection.ftcPose.yaw, detection.ftcPose.pitch, detection.ftcPose.roll));
                telemetry.addLine(String.format("Range %6.2f inches", detection.ftcPose.range));
                telemetry.addLine(String.format("Bearing %6.2f degrees", detection.ftcPose.bearing));

                //Pose ftcStandard = PoseConverter.pose2DToPose(new Pose2D(DistanceUnit.INCH, detection.ftcPose.x,detection.ftcPose.y, AngleUnit.DEGREES, Math.toRadians(detection.ftcPose.yaw) ), FTCCoordinates.INSTANCE);
                //return ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                //Pose cameraToTagPose = new Pose(detection.ftcPose.x,detection.ftcPose.y,detection.ftcPose.yaw, InvertedFTCCoordinates.INSTANCE);
                //cameraToTagPose = cameraToTagPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                Pose2D CameraPose = new Pose2D(DistanceUnit.INCH,detection.ftcPose.x,detection.ftcPose.y, AngleUnit.RADIANS, Math.toRadians(detection.ftcPose.yaw));
                Pose pedroPose = PoseConverter.pose2DToPose(CameraPose,InvertedFTCCoordinates.INSTANCE);
                pedroPose = pedroPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                return pedroPose;


            } else {
                // This happens if an unknown tag is seen.
                telemetry.addLine("\n(First detection is an unknown tag)");

                return follower.getPose();
            }
        } else {
            // No tags are currently visible.
            telemetry.addLine("\nNo AprilTags Detected");

            return follower.getPose();
        }



    }

}


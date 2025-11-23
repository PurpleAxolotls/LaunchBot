/*
   Program Purpose:

   Version of Robot with complete functionality added, however with complete manual control
   over output RPM.

 */

package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;



import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Configurable
@TeleOp(name="PIDTuner")
public class PIDTuningOpMode extends OpMode {

    static double flyWheelTargetVelocity = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance(); // Add this line

    FlyWheelPID control = new FlyWheelPID(0.05,0,0);

    boolean dpad_up_pressed_previous = false;
    boolean dpad_down_pressed_previous = false;
    boolean a_pressed_previous = false;

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
        intake.setDirection(CRServo.Direction.FORWARD);

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
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Gate
        gate = hardwareMap.get(Servo.class, "gate");
    }

    double fallbackRPM = 2000;

    @Override
    public void loop() {

        if (gamepad2.a && !a_pressed_previous) {
            gateLogic();
        }
        a_pressed_previous = gamepad2.a;

        wheelLogic();
        flyWheelLogic();
        chamberLogic();
        intake.setPower(-1);

        TelemetryPacket packet = new TelemetryPacket();

        // Add data to the packet for FTC Dashboard
        packet.put("Target RPM", flyWheelDesiredRPM);
        packet.put("Actual RPM Left", (leftFlyWheel.getVelocity() * 60) / flywheelTPR);
        packet.put("Actual RPM Right", (rightFlyWheel.getVelocity() * 60) / flywheelTPR);
        packet.put("current velocity",leftFlyWheel.getVelocity());
        packet.put("target velocity", flyWheelTargetVelocity);

        // Send the packet to the dashboard
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Target RPM", flyWheelDesiredRPM);
        telemetry.addData("Actual RPM Left", "%.2f", (leftFlyWheel.getVelocity() * 60) / flywheelTPR);
        telemetry.addData("Actual RPM Right", "%.2f", (rightFlyWheel.getVelocity() * 60) / flywheelTPR);

    }

    // Methods
    public void wheelLogic() {

        if (gamepad1.dpad_down) {
            axial = -.25;
            lateral = 0;
            yaw = 0;
        } else if (gamepad1.dpad_up) {
            axial = .25;
            lateral = 0;
            yaw = 0;
        } else if (gamepad1.dpad_left) {
            axial = 0;
            lateral = -.25;
            yaw = 0;
        } else if (gamepad1.dpad_right) {
            axial = 0;
            lateral = .25;
            yaw = 0;
        } else {
            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;
        }

        frontLeftPower  = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;
        backLeftPower   = axial - lateral + yaw;
        backRightPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

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


        double command = control.update(flyWheelTargetVelocity, leftFlyWheel.getVelocity());
        leftFlyWheel.setPower(command);
        rightFlyWheel.setPower(command);

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

}


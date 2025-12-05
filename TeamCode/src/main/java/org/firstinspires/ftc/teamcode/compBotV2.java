/*
   Program Purpose:

   Version of Robot with complete functionality added, however with complete manual control
   over output RPM.

 */

package org.firstinspires.ftc.teamcode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="compBotV2")
public class compBotV2 extends OpMode {

    boolean dpad_up_pressed_previous = false;
    boolean dpad_down_pressed_previous = false;
    boolean a_pressed_previous = false;

    DcMotor chamberMotor = null;
    CRServo upperRightChamber = null;
    CRServo specialChamber = null;
    CRServo backRightChamber = null;

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

    Servo light1 = null;
    Servo light2 = null;

    private Timer opModeTimer;

    @Override
    public void init() {

        // Chamber
        upperRightChamber = hardwareMap.get(CRServo.class, "frontRightS");
        specialChamber = hardwareMap.get(CRServo.class, "specialChamber");
        chamberMotor = hardwareMap.get(DcMotor.class, "chamberMotor");
        backRightChamber = hardwareMap.get(CRServo.class, "backRightChamber");

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
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Gate
        gate = hardwareMap.get(Servo.class, "gate");

        light1 = hardwareMap.get(Servo.class, "light1");
        light2 = hardwareMap.get(Servo.class, "light2");

        opModeTimer = new Timer();

    }

    double fallbackRPM = 2000;

    public void start() {
        light2.setPosition(1);
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {

        if (gamepad2.a && !a_pressed_previous) {
            gateLogic();
        }
        a_pressed_previous = gamepad2.a;

        wheelLogic();
        flyWheelLogic();
        chamberLogic();
        lightLogic((flyWheelDesiredRPM / 60) * flywheelTPR);

        intake.setPower(-1);

        telemetry.addData("Target RPM", flyWheelDesiredRPM);
        telemetry.addData("Actual RPM Left", "%.2f", (leftFlyWheel.getVelocity() * 60) / flywheelTPR);
        telemetry.addData("Actual RPM Right", "%.2f", (rightFlyWheel.getVelocity() * 60) / flywheelTPR);
        telemetry.update();

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
        backRightChamber.setPower(1);
        chamberMotor.setPower(-1);
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

    public void lightLogic(double flyWheelTargetVelocity) {
        if (Math.abs(flyWheelTargetVelocity - leftFlyWheel.getVelocity()) > 21) {
            light1.setPosition(.277);
        } else {light1.setPosition(.444);}


        if(opModeTimer.getElapsedTimeSeconds() > 30)
        {
            light2.setPosition(.333);
        }
    }


}


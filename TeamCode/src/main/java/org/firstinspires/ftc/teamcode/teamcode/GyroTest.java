package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "GyroTest", group = "Auton")
public class GyroTest extends LinearOpMode {

    //Declare Objects
    private DcMotorEx motorLeft;
    private DcMotorEx motorRight;
    private DcMotorEx motorMiddle;
    private ColorSensor colorsensor;
   // private ColorSensor colorSensorFront;
    private Servo leftServo;
    private Servo rightServo;
    public static DcMotorEx fourBar;
    BNO055IMU imu;


    static final double maxPosition = 0.0;
    static final double minPosition = 1.0;


    double NEW_P = 12.00;
    double NEW_I = 0.03;
    double NEW_D = 0;
    double New_F = 0;

    static final double armSpeed = 1;

    static final int ToleranceAd = 20;

    boolean stop = false;


    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorLeft");
        motorRight = (DcMotorEx) hardwareMap.dcMotor.get("motorRight");
        motorMiddle = (DcMotorEx) hardwareMap.dcMotor.get("motorMiddle");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        colorsensor = hardwareMap.colorSensor.get("colorsensor");
        //colorSensorFront = hardwareMap.colorSensor.get("colorSensorFront");
        fourBar = (DcMotorEx) hardwareMap.get(DcMotor.class, "lifter");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorMiddle.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorMiddle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        colorsensor.enableLed(true);
     // colorSensorFront.enableLed(true);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.

        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, New_F);
        motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                pidNew.p, pidNew.i, pidNew.d);
        telemetry.update();


        waitForStart();
       movePlatform(maxPosition);
     moveArm(125 + ToleranceAd, armSpeed, 25);
       colorSensorFront();
      movePlatform(minPosition);
       sleep(1000);
       driveForward(-6000,-1);
       sleep(2000);
       movePlatform(maxPosition);
      driveSideways(-3200, -1);
        sleep(2000);
        driveForward(32/00,1);
        sleep(2000);
        driveSideways(3200, 1);
        sleep(2000);
        driveForward(-3000,-1);
        colorSensor();


        telemetry.addData("Mode", "running");
        telemetry.update();


    }


    private void olddriveForward(int distance, int tolerance, double speed) {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("This kinda works", "kinda");
        telemetry.update();

        motorLeft.setTargetPosition(distance);
        motorRight.setTargetPosition(distance);


        while ((Math.abs(motorLeft.getCurrentPosition()) < Math.abs(motorLeft.getTargetPosition()) - tolerance) && (Math.abs(motorRight.getCurrentPosition()) < Math.abs(motorRight.getTargetPosition()) - tolerance)) {
            motorLeft.setPower(speed);
            motorRight.setPower(speed);
            telemetry.addData("PID IS WORKIN'", "0");
            telemetry.addData("Current Position:", motorLeft.getCurrentPosition());
            telemetry.update();
        }

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Current Position:", motorLeft.getCurrentPosition());
        telemetry.addData("Done Moving", " ");
        telemetry.update();

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("status: ", "setting power to zero");

        motorLeft.setPower(0);
        motorRight.setPower(0);

        telemetry.addData("status: ", "done going forward");
    }
    private void colorSensor() {
        while (stop == false) {
            if (!(colorsensor.red() > 20) && stop == false) {
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double angle = angles.firstAngle;
                motorLeft.setPower(.05 * angle);
                motorRight.setPower(- .05 * angle);
                motorMiddle.setPower(1);
            } else {
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                motorMiddle.setPower(0.0);
                stop = true;
            }
        }

    }
    private void colorSensorFront() {
        while (stop == false) {
            if (!(colorsensor.red() > 25) && stop == false) {
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double angle = angles.firstAngle;
                motorLeft.setPower(.75);
                motorRight.setPower(.75);
                telemetry.addData("Blue", colorsensor.blue());
                telemetry.update();
            } else {
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
                stop = true;
            }
        }
    }
    private void movePlatform(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
        telemetry.addData("platform moved", "complete");
    }
    private void driveForward(double distance, double power) {
        telemetry.addData("Moving", "");
        telemetry.update();
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(motorLeft.getCurrentPosition()) < Math.abs(distance) && Math.abs(motorRight.getCurrentPosition()) < Math.abs(distance)) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angle = angles.firstAngle;
            motorLeft.setPower(power + .2 * angle);
            motorRight.setPower(power - .2 * angle);
        }
        telemetry.addData("Done Moving", "");
        telemetry.update();
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
    private void driveSideways(double distance, double power) {
        telemetry.addData("Moving", "");
        telemetry.update();
        motorMiddle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMiddle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(motorMiddle.getCurrentPosition()) < Math.abs(distance)) {
            motorMiddle.setPower(power);
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angle = angles.firstAngle;
            motorLeft.setPower(.05 * angle);
            motorRight.setPower(- .05 * angle);
        }
        motorMiddle.setPower(0);
        telemetry.addData("Done Moving", "");
        telemetry.update();
    }
    public void armEncoderReset() {
        fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void moveArm(int position, double power, int tolerance) {
        if (fourBar.getCurrentPosition() > 200) {
        } else {
            armEncoderReset();

            fourBar.setPower(power);

            fourBar.setTargetPositionTolerance(tolerance + ToleranceAd);

            fourBar.setTargetPosition(position);

            fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (fourBar.isBusy()) {
                telemetry.addData("Moving to", position);
                telemetry.addData("From", fourBar.getCurrentPosition());
                telemetry.addData("Power", fourBar.getPower());
                telemetry.update();
            }
            telemetry.addData("Arm Status", "done," + position);
            telemetry.update();
            fourBar.setPower(0);
        }
    }
}

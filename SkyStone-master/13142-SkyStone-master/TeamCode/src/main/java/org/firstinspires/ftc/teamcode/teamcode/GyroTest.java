package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
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
    private DcMotorEx  motorRight;
    private ColorSensor colorsensor;
    private Servo leftServo;
    private Servo rightServo;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
   PIDcontroller pidRotate;


    //Declare Variables
    double globalAngle;
    double power = 1;
    double correction;
    double rotation;

    static final double maxPosition = 0.0;
    static final double minPosition = 1.0;


    double NEW_P= 12.00;
    double NEW_I =0.03;
    double NEW_D = 0;
    double New_F = 0;

   boolean stop = false;



    @Override
            public void runOpMode()
    {
        motorLeft = (DcMotorEx)hardwareMap.dcMotor.get("motorLeft");
        motorRight =(DcMotorEx)hardwareMap.dcMotor.get("motorRight");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        colorsensor = hardwareMap.colorSensor.get("colorsensor");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        colorsensor.enableLed(true);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDcontroller(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
       // pidDrive = new PIDcontroller(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

       PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, New_F);
       motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
       motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                pidNew.p, pidNew.i, pidNew.d);
        telemetry.update();



        waitForStart();




        //Start autonomous code
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        movePlatform(maxPosition);
        sleep(1000);
       driveForward(2500, 45, 1);
        sleep(1000);
        movePlatform(minPosition);
       sleep(1000);
       driveForward(-2400, 100, 1);
       sleep(1000);
        rotate(25, power);
       sleep(1000);
        driveForward(-200, 40, 1);
        sleep(1000);
       movePlatform(maxPosition);
        sleep(1000);
        rotate(70,power);


        telemetry.addData("Mode", "running");
        telemetry.update();



    }
    private void driveForward(int distance, int tolerance, double speed)
    {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("This kinda works", "kinda");
        telemetry.update();

        motorLeft.setTargetPosition(distance);
        motorRight.setTargetPosition(distance);

        motorLeft.setPower(speed);
        motorRight.setPower(speed);

        while(Math.abs(motorLeft.getCurrentPosition()) < Math.abs( motorLeft.getTargetPosition()) -  tolerance)
        {
            telemetry.addData( "PID IS WORKIN'", "0");
            telemetry.addData("Current Position:", motorLeft.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Current Position:", motorLeft.getCurrentPosition());
        telemetry.update();

         motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("status: ","setting power to zero");

        motorLeft.setPower(0);
        motorRight.setPower(0);

        telemetry.addData("status: ","done going forward");
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;// yo neccisitas ayuda, por favor.

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                motorLeft.setPower(power);
                motorRight.setPower(-power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                motorLeft.setPower(-power);
                motorRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                motorLeft.setPower(-power);
                motorRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        motorLeft.setPower(0);
        motorRight.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void colorSensor()
    {
        while(stop == false)
        if(!(colorsensor.blue()>70)&& stop == false )
        {
            motorLeft.setPower(-1.0);
            motorRight.setPower(1.0);
            telemetry.addData("Blue", colorsensor.blue());
            telemetry.update();
        }
        else {
            motorLeft.setPower(0.0);
            motorRight.setPower(0.0);
            stop = true;
        }

    }
    private void movePlatform(double position)
    {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
        telemetry.addData("platform moved", "complete");
    }
}

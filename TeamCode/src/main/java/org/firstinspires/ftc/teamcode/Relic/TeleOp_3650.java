package org.firstinspires.ftc.teamcode.Relic;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Operation Telly Relic", group = "3650 Prod")
public class TeleOp_3650 extends OpMode{
    private DcMotor lDrive, rDrive, lift1;
    private Servo armServo, grabber;

    boolean motorBusy = false;
    IMU_class imu;
    double initialHeading;


    @Override
    public void init() {
        imu = new IMU_class("imu", hardwareMap);
        initialHeading = getHeading(imu);
        rDrive = hardwareMap.dcMotor.get("rDrive");

        lDrive = hardwareMap.dcMotor.get("lDrive");
        lDrive.setDirection(DcMotor.Direction.REVERSE);

        lift1 = hardwareMap.dcMotor.get("lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        armServo = hardwareMap.servo.get("armServo");
        armServo.setPosition(0.5);
        grabber = hardwareMap.servo.get("grabber");
        grabber.setPosition(0.5);

    }

    @Override
    public void loop() {

        if (gamepad2.a){
            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.x){ // Spin 180ish degrees with one button
            motorBusy = true;
            //MIGHT need to thread this
            turn2Angle(-179, imu);
        }
        else if(!lDrive.isBusy()){
            motorBusy = false;
        }

        if (!motorBusy) {
            if (gamepad1.a) {
                lDrive.setPower(.6);
                rDrive.setPower(.6);
            } else if (gamepad1.y) {
                lDrive.setPower(-.6);
                rDrive.setPower(-.6);
            } else if (gamepad1.b) {  // full forward
                lDrive.setPower(-.99);
                rDrive.setPower(-.99);
            } else {
                lDrive.setPower(gamepad1.left_stick_y * .6);
                rDrive.setPower(gamepad1.right_stick_y * .6);
            }
        }

        // lift
        if (gamepad2.right_trigger > 0.1){
            lift1.setPower(gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > 0.1){
            lift1.setPower(-gamepad2.left_trigger);
        }

        else{
            lift1.setPower(0);
        }

        // armServo
        if (gamepad1.dpad_up) {
            armServo.setPosition(.75);
        }
        else if (gamepad1.dpad_down){
            armServo.setPosition(.1);
        }

        // Grabber servo (Need to test values!)
        if (gamepad2.dpad_left){
            // close
            grabber.setPosition(0.2);
        }
        else if (gamepad2.dpad_right){
            // open
            grabber.setPosition(0.8);
        }
        else{
            // stop
            grabber.setPosition(0.5);
        }

        telemetry.addData("Encoder value: ", lift1.getCurrentPosition());
        telemetry.addData("Servo value: ", armServo.getPosition());

    }
    void turn2Angle(double target, IMU_class i){
        lDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initialHeading = getHeading(i);
        double relTarget = initialHeading + target;
        if (0 > target){
            //Turn right
            lDrive.setPower(.7);
            rDrive.setPower(-.7);
            while (Math.abs(relTarget - getHeading(i)) > 5){
                telemetry.addData("degrees to target", Math.abs(getHeading(i) - relTarget));
                telemetry.addData("current heading", getHeading(i));
                telemetry.update();
            }
        }
        else{
            //Turn left
            lDrive.setPower(-.7);
            rDrive.setPower(.7);
            while (Math.abs(relTarget - getHeading(i)) > 5){
                telemetry.addData("degrees to target", Math.abs(getHeading(i) - relTarget));
                telemetry.addData("current heading", getHeading(i));
                telemetry.update();
            }
        }
        setBothPower(0.0);
    }
    double getHeading(IMU_class a){
        return a.getAngles()[0];
    }
    void setBothPower(double power){
        lDrive.setPower(power);
        rDrive.setPower(power);
    }

}

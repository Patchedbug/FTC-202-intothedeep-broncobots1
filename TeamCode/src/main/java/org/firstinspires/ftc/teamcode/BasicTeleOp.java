package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BasicTeleOp.RobotHardware;

@TeleOp(name="TeleOp", group="Iterative Opmode")
public class BasicTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2;

        // Variables
        double ClawOffset = 0; // offsets Claw starting position


        double Wrist1target = robot.Wrist1.getPosition();
        double Wrist0target = robot.Wrist0.getPosition();

        //double Wristtarget =0;
        robot.speed =2;
        ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double FrameRate = (runTime.time()*1000);
            //runTime.reset();
            double pointOneSec = runTime.time();
            double isEvenSec = pointOneSec % 100;
            double time = getRuntime();
            double tdist;
            boolean override =false;



            // gamepad 1 controls

            // Drivetrain

            //slow fast toggles

            //b slows down
            //y speed up




            //actual driving command
            robot.driveWithControllers(Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x,
                    -1 * Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y,
                    Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x, 1, robot.slowFast(gamepad1.b, gamepad1.y));


            /*Overide to allow gamepad 1 to control upper and lower arm
            Intended in the case that the robot flips over
            hold x and a to enable
            When enabled, gamepad 1 controls drivetrain the same but also controls lower arm with right bumper and trigger,
            and upper arm with left bumper and trigger
             */

            if (gamepad1.a && gamepad1.x && !gamepad1.y && !gamepad1.b){
                override = true;
            }

            robot.armOverrideControl(gamepad1.right_bumper, gamepad1.right_trigger, gamepad1.left_bumper, gamepad1.left_trigger, override);
            //Gamepad 2 controls

            // Intake
            robot.grabberControl(gamepad2.right_bumper, gamepad2.left_bumper);


            // Arm
            robot.armLowerControl(gamepad2.right_stick_y);


            // upper arm
            robot.armUpperControl(gamepad2.left_stick_y);


            //wrist
            robot.wristControl(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.x, gamepad2.b, gamepad2.y);
            telemetry.addData("Armlower(right)", robot.Arm0.getCurrentPosition());
            telemetry.addData("Armupper", robot.ArmUpper.getCurrentPosition());

        }
    }

    static class RobotHardware{


        public HardwareMap map;


        public final Telemetry telemetry;
        static final double EncoderTickToTile = 1 / ((23.4 * 25.4 * 4096) / 109.955742876);


        public final BNO055IMU imu;


        public final DcMotor RF, RB, LF, LB;


        //public final DcMotor Climb, ArmUpper, Arm0, Arm1;
        public final DcMotor ArmUpper, Arm0, Arm1;

        public final Servo Grab;

        public final Servo Wrist0, Wrist1;

        public int StartingArmPosition0;
        public int ArmHoldPosition0;
        public int StartingArmPosition1;
        public int ArmHoldPosition1;

        public int StartingArmUpperPosition;
        public int ArmUpperHoldPosition;
        public int speed;


        //c1 and C2 are approx
        //  static final double C1 = 2.72824076488, C2 = 1.82212373908, mmToTile =
        //        0.00168248199744, EncoderTickToMM = 0.026844663788,
        //Encoderticktomm is constant and fixed- no calibration
        //mmtotile is constant + fixed- no calibration
        //C2 is updated
        //C1 is not used agian
        //encoderBad = 1, // (tiles supposed to go * ticks supposed to go) / (distance told to go * encoder ticks actually gone)
        //amount of error in encoder for calibration
        //needs to be updated to our robot
        // encoderTicksPer360 = 3.11784486041 * 22708.224; // encoder wheel distance for 360 degrees rotation in tile lengths * tile to encoder
        //1st term is maybe updated
        //2nd term is updated to our encoders

        public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
            this.telemetry = telemetry;
            this.map = hardwareMap;


            // "deviceName:" is what appears in the configure section on control hub
            RF = hardwareMap.get(DcMotor.class, "RF");
            RB = hardwareMap.get(DcMotor.class, "RB");
            LF = hardwareMap.get(DcMotor.class, "LF");
            LB = hardwareMap.get(DcMotor.class, "LB");
            //Climb = hardwareMap.get(DcMotor.class, "Climb");
            ArmUpper = hardwareMap.get(DcMotor.class, "Armupper");
            Arm0 = hardwareMap.get(DcMotor.class, "Arm0");
            Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
            Grab = hardwareMap.get(Servo.class, "Grab");
            Wrist0 = hardwareMap.get(Servo.class, "Wrist0");
            Wrist1 = hardwareMap.get(Servo.class, "Wrist1");


            //invert specific motors
            RF.setDirection(DcMotor.Direction.FORWARD);
            RB.setDirection(DcMotor.Direction.FORWARD);
            LF.setDirection(DcMotor.Direction.REVERSE);
            LB.setDirection(DcMotor.Direction.REVERSE);
            //Climb.setDirection(DcMotor.Direction.REVERSE);
            ArmUpper.setDirection(DcMotor.Direction.FORWARD);
            Arm0.setDirection(DcMotorSimple.Direction.REVERSE);
            Arm1.setDirection(DcMotorSimple.Direction.REVERSE);

            //Grab.setDirection(CRServo.Direction.FORWARD);
            //Wrist.setDirection(CRServo.Direction.FORWARD);


            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm0.setTargetPosition(Arm0.getCurrentPosition());
            Arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm1.setTargetPosition(Arm1.getCurrentPosition());
            Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //Climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ArmUpper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            ArmUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Arm0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            StartingArmPosition0 = Arm0.getCurrentPosition();
            ArmHoldPosition0 = Arm0.getCurrentPosition();
            StartingArmPosition1 = Arm1.getCurrentPosition();
            ArmHoldPosition1 = Arm1.getCurrentPosition();

            StartingArmUpperPosition = ArmUpper.getCurrentPosition();
            ArmUpperHoldPosition = ArmUpper.getCurrentPosition();
        }

        public void driveWithControllers(double strafe, double forward, double turn, double throttle, int speed) {
            double max_power = Math.max(1, Math.max(Math.max(
                    Math.abs(-forward - strafe + turn), // LF
                    Math.abs(-forward + strafe + turn) // LB
            ), Math.max(
                    Math.abs(-forward + strafe - turn), // RF
                    Math.abs(-forward - strafe - turn) // RB
            )));
            strafe /= max_power;
            forward /= max_power;
            turn /= max_power;
            throttle = 0.75;
            if (speed == 1) {
                throttle *= 0.66666666666;
            }
            if (speed == 3) {
                throttle *= 1.33333333333;
            }
            LF.setPower(throttle * (forward - strafe + turn));
            LB.setPower(throttle * (forward + strafe + turn));
            RF.setPower(throttle * (forward + strafe - turn));
            RB.setPower(throttle * (forward - strafe - turn));


        }

        public int slowFast(boolean s, boolean f) {

            if (s) {
                if (speed != 1) {
                    speed = 1;
                } else {
                    speed = 2;
                }
                // y speeds up
            } else if (f) {
                if (speed != 3) {
                    speed = 3;
                } else {
                    speed = 2;
                }
            }
            return speed;
        }

        public void armOverrideControl(boolean armLowerUp, float armLowerDown, boolean armUpperUp, float armUpperDown, boolean override) {
            while (override) {

                if (armLowerUp) {
                    Arm0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Arm0.setPower(-0.25);
                    Arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Arm1.setPower(0.25);

                } else if (armLowerDown > 0.05) {
                    Arm0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Arm0.setPower(-0.5);
                    Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Arm1.setPower(-0.5);

                } else if (armUpperUp) {
                    ArmUpper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    ArmUpper.setPower(0.5);
                } else if (armUpperDown > 0.05) {
                    ArmUpper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    ArmUpper.setPower(-0.5);
                }
            }
        }

        //lower arm control

        public void armLowerControl(double rightystick) {
            if (Math.abs(rightystick) > 0.05) {
                Arm0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Arm0.setPower(-0.5 * rightystick / 4);
                Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Arm1.setPower(0.5 * rightystick / 4);
                ArmHoldPosition0 = Arm0.getCurrentPosition();
                ArmHoldPosition1 = Arm1.getCurrentPosition();
            } else {
                Arm0.setPower(-1);
                Arm0.setTargetPosition(ArmHoldPosition0 + StartingArmPosition0);
                Arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm1.setPower(1);
                Arm1.setTargetPosition(ArmHoldPosition1 + StartingArmPosition1);
                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        //upper arm control

        public void armUpperControl(double leftystick) {
            if (Math.abs(leftystick) > 0.1) {
                ArmUpper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ArmUpper.setPower(-leftystick / 3);
                ArmUpperHoldPosition = ArmUpper.getCurrentPosition();
            } else {
                ArmUpper.setPower(1);
                ArmUpper.setTargetPosition(ArmUpperHoldPosition + StartingArmUpperPosition);
                ArmUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        // grabber control
        public final double ClawOffset = 0;
        public void grabberControl(boolean lbump, boolean rbump) {
            if (lbump) Grab.setPosition(0 + ClawOffset);
            else if (rbump) Grab.setPosition(0.2 + ClawOffset);
        }


        //new dif wrist
        //300 degree servo so to get the right number
            // degrees/300
        //init wrist is at 50degrees/ 0.4333 off of what should be zero in the servo language
        public double wristadjust= 0.4333;
        public double twist=0;
        public double updown = 0;
        public int wristpos = 0;
        //0 is initial, 1 is up, 2 is middle, 3 is down
        //wrist initial is 0 for wrist0 and 1 for wrist1
        public void wristControl(boolean twistto90, boolean twistto0, boolean wristup, boolean wristdown, boolean wristmid ) {

            if (wristmid && wristpos <2){
                Wrist1.setDirection(Servo.Direction.REVERSE);
                Wrist0.setDirection(Servo.Direction.FORWARD);
                Wrist1.setPosition(0.5);
                Wrist0.setPosition(0.5);
            } else if (wristmid && wristpos >=2) {
                Wrist1.setDirection(Servo.Direction.FORWARD);
                Wrist0.setDirection(Servo.Direction.REVERSE);
                Wrist1.setPosition(0.5);
                Wrist0.setPosition(0.5);
            }
            if (wristup && wristpos<1){
                Wrist0.setDirection(Servo.Direction.REVERSE);
                Wrist1.setDirection(Servo.Direction.FORWARD);
                Wrist0.setPosition(0.3);
                Wrist1.setPosition(0.9);
            } else if (wristup && wristpos>=1) {
                Wrist0.setDirection(Servo.Direction.FORWARD);
                Wrist1.setDirection(Servo.Direction.REVERSE );
                Wrist0.setPosition(0.3);
                Wrist1.setPosition(0.9);
            }
            if (wristdown){
                Wrist0.setDirection(Servo.Direction.REVERSE);
                Wrist1.setDirection(Servo.Direction.FORWARD);
                Wrist0.setPosition(0.3);
                Wrist1.setPosition(0.9);
            }
            if (twistto90 && twist==0){
                Wrist0.setDirection(Servo.Direction.FORWARD);
                Wrist1.setDirection(Servo.Direction.FORWARD);
                Wrist0.setPosition(0.5+Wrist0.getPosition());
                Wrist1.setPosition(0.5-Wrist1.getPosition());
                twist= 0.5;

            }
            if (twistto0 && twist==0.5){
                Wrist0.setDirection(Servo.Direction.REVERSE);
                Wrist1.setDirection(Servo.Direction.REVERSE);
                Wrist0.setPosition(0+Wrist0.getPosition());
                Wrist1.setPosition(0-Wrist1.getPosition());
                twist= 0;

            }
        }



    }


}


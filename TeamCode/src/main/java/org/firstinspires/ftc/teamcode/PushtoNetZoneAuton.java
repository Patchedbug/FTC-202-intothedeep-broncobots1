package org.firstinspires.ftc.teamcode;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "Push to Net Zone", group = "Autonomous")
public abstract class PushtoNetZoneAuton extends LinearOpMode {
    //lower arm
    public int armlower0hangpos= 1;
    public int armlower1hangpos= 1;


    //set up the command base for the lower arm
    //TODO Add the pick up from human player position+ any other positions
    public class armLower {
        private DcMotorEx Arm0, Arm1;

        public armLower(HardwareMap hardwareMap) {
            DcMotor Arm0 = hardwareMap.get(DcMotor.class, "Arm0");
            DcMotor Arm1 = hardwareMap.get (DcMotor.class, "Arm1");
            Arm0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm0.setDirection(DcMotorSimple.Direction.REVERSE);
            Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm1.setDirection(DcMotorSimple.Direction.REVERSE);
            Arm0.setTargetPosition(Arm0.getCurrentPosition());
            Arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm1.setTargetPosition(Arm1.getCurrentPosition());
            Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //put the lower arm into position to score a specimen on the high bar
        public class armLowerToScoreOnBar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Arm0.setTargetPosition(armlower0hangpos);
                Arm1.setTargetPosition(armlower1hangpos);

                return false;
            }
        }
        public Action armLowerToScoreOnBar() {
            return new armLowerToScoreOnBar();
        }
    }



    //set up the command base for the upper arm
    //TODO add specific commands to score specimen on high bar
    public class armUpper {
        private DcMotorEx ArmUpper;

        public armUpper(HardwareMap hardwareMap) {
            DcMotor ArmUpper = hardwareMap.get(DcMotor.class, "armUpper");

            ArmUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ArmUpper.setDirection(DcMotorSimple.Direction.FORWARD);
            ArmUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmUpper.setTargetPosition(ArmUpper.getCurrentPosition());
        }
    }



    // Command base for claw
    //TODO add specific commands to score specimen on high bar
    public class claw {
        private Servo claw;

        public claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "Grab");
        }
    }

    //TODO Add a wrist command base



    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(38, 63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder pushToNetZone = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(56,63))
                .strafeTo(new Vector2d(32, 63))
                .strafeTo(new Vector2d(40, 15))
                .strafeTo(new Vector2d(44, 15))
                .strafeTo(new Vector2d(54, 59))
                .strafeTo(new Vector2d(54, 59))
                .strafeTo(new Vector2d(40, 48))
                .strafeTo(new Vector2d(47, 15))
                .strafeTo(new Vector2d(56, 15))
                .strafeTo(new Vector2d(56, 57))
                .strafeTo(new Vector2d(56, 15))
                .strafeTo(new Vector2d(63, 15))
                .strafeTo(new Vector2d(63, 57))
                .strafeTo(new Vector2d(25, 10));


        while (!isStopRequested() && !opModeIsActive()) {

        }
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(
                pushToNetZone.build()
                /*new SequentialAction(
                        go,


                )*/
        );

    }
}


package org.firstinspires.ftc.teamcode;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.location.imu.MultiIMU;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.Motor;
import com.epra.epralib.ftclib.movement.frames.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.movement.frames.ServoFrame;
import com.epra.epralib.ftclib.movement.pid.PIDController;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Autonomous(name = "BioBuzzRedAuto", group = "Autonomous")
public class BioBuzzRedAuto extends LinearOpMode {

    private final Pose START_POSE = new Pose(new Vector(0, 0), new Angle());

    private MotorController frontLeft;
    private MotorController frontRight;
    private MotorController backLeft;
    private MotorController backRight;
    private DriveTrain drive;

    private HashMap<String, MotorController> nonDriveMotors;

    private MultiIMU imu;
    private Odometry odometry;

    private Controller controller1;
    private Controller controller2;

    @Override
    public void runOpMode() {

        //Initializes the LogController
        LogController.init();

        //Setting up the IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        //Setting up the IMU
        /*IMU tempIMU = hardwareMap.get(IMU.class, "imu 1");
        tempIMU.initialize(new IMU.Parameters(orientationOnRobot));
        imu = new MultiIMU.Builder(new IMUFrame(tempIMU))
                .initialYaw(Geometry.subtract(Angle.degree(tempIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), START_POSE.angle))
                .loggingTarget(MultiIMU.Axis.YAW)
                .build();
        LogController.addLogger(imu); */

        //Setting up the MotorControllers for the DriveTrain
        frontRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "frontRight")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_FRONT)
                .build();
        backRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "backRight")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_BACK)
                .build();
        frontLeft = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "frontLeft")))
                .driveOrientation(DriveTrain.Orientation.LEFT_FRONT)
                .direction(Motor.Direction.REVERSE)
                .build();
        backLeft = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "backLeft")))
                .driveOrientation(DriveTrain.Orientation.LEFT_BACK)
                .direction(Motor.Direction.REVERSE)
                .build();

        //Setting up the Odometry
        odometry = new Odometry.Builder()
                .leftEncoder(frontLeft::getCurrentPosition, 0.01, new Vector(8, 4))
                .rightEncoder(backLeft::getCurrentPosition, 0.01, new Vector(-8, 4))
                .perpendicularEncoder(frontRight::getCurrentPosition, 0.01, new Vector(0, 2))
                //.heading(imu::getYaw)
                .startPose(new Pose(new Vector(0, 0), Angle.degree(0)))
                .loggingTargets(Odometry.LoggingTarget.X, Odometry.LoggingTarget.Y)
                .build();
        LogController.addLogger(odometry);

        //Initializing the DriveTrain
        drive = new DriveTrain.Builder()
                .motor(frontRight)
                .motor(frontLeft)
                .motor(backRight)
                .motor(backLeft)
                .driveType(DriveTrain.DriveType.MECANUM)
                .build();

        //Setting up the MotorControllers that are not part of the DriveTrain
        nonDriveMotors = new HashMap<>();
        //Add MotorControllers like so:
        nonDriveMotors.put("pollenShooter",
                new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "pollenShooter")))
                        .id("pollenShooter")
                        .addLogTarget(MotorController.LogTarget.POSITION)
                        .build());
        LogController.addLogger(nonDriveMotors.get("pollenShooter"));

        /* nonDriveMotors.put("nectarShooter",
                new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "nectarShooter")))
                        .id("nectarShooter")
                        .addLogTarget(MotorController.LogTarget.POSITION)
                        .build());
        LogController.addLogger(nonDriveMotors.get("nectarShooter")); */

        nonDriveMotors.put("Intake",
                new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "Intake")))
                        .id("Intake")
                        .addLogTarget(MotorController.LogTarget.POSITION)
                        .build());
        LogController.addLogger(nonDriveMotors.get("Intake"));

        //Setting up controller1
        controller1 = new Controller(gamepad1, 0.0f, "1",
                new Controller.Key[]{
                        Controller.Key.LEFT_STICK_X,
                        Controller.Key.LEFT_STICK_Y,
                        Controller.Key.RIGHT_STICK_X,
                        Controller.Key.RIGHT_STICK_Y
                });

        //Setting up controller2
        controller2 = new Controller(gamepad2, 0.0f, "2",
                new Controller.Key[]{
                        Controller.Key.LEFT_STICK_X,
                        Controller.Key.LEFT_STICK_Y,
                        Controller.Key.RIGHT_STICK_X,
                        Controller.Key.RIGHT_STICK_Y
                });

        LogController.logInfo("Waiting for start...");
        waitForStart();
        LogController.logInfo("Starting Autonomous.");
        //What the robot does while running

        while (opModeIsActive()) {
            LogController.logData();
            //Updates all active PID loops
            PIDController.update();



        }

        //Closes all logs n
        LogController.logInfo("Autonomous complete.");
        LogController.closeLogs();
    }
}
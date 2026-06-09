package org.firstinspires.ftc.teamcode;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.location.imu.IMUFrame;
import com.epra.epralib.ftclib.location.imu.MultiIMU;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.Motor;
import com.epra.epralib.ftclib.movement.frames.CRServoFrame;
import com.epra.epralib.ftclib.movement.frames.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.movement.frames.ServoFrame;
import com.epra.epralib.ftclib.movement.pid.PIDController;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;

@TeleOp(name = "BioBuzzTeleOp", group = "TeleOp")
public class BioBuzzTeleOp extends LinearOpMode {

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
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        //Setting up the IMU
        IMU tempIMU = hardwareMap.get(IMU.class, "imu 1");
        tempIMU.initialize(new IMU.Parameters(orientationOnRobot));
        imu = new MultiIMU.Builder(new IMUFrame(tempIMU))
                .initialYaw(Geometry.subtract(Angle.degree(tempIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), START_POSE.angle))
                .loggingTarget(MultiIMU.Axis.YAW)
                .build();
        LogController.addLogger(imu);

        //Setting up the MotorControllers for the DriveTrain
        frontRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northeastMotor")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_FRONT)
                .build();
        backRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southeastMotor")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_BACK)
                .build();
        frontLeft = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northwestMotor")))
                .driveOrientation(DriveTrain.Orientation.LEFT_FRONT)
                .direction(Motor.Direction.REVERSE)
                .build();
        backLeft = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southwestMotor")))
                .driveOrientation(DriveTrain.Orientation.LEFT_BACK)
                .direction(Motor.Direction.REVERSE)
                .build();

        //Setting up the Odometry
        odometry = new Odometry.Builder()
                .leftEncoder(frontLeft::getCurrentPosition, 0.01, new Vector(8, 4))
                .rightEncoder(backLeft::getCurrentPosition, 0.01, new Vector(-8, 4))
                .perpendicularEncoder(frontRight::getCurrentPosition, 0.01, new Vector(0, 2))
                .heading(imu::getYaw)
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
        nonDriveMotors.put("Intake",
        new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "Intake")))
                .id("Intake")
                .addLogTarget(MotorController.LogTarget.POSITION)
                .build());
         LogController.addLogger(nonDriveMotors.get("Intake"));

        //Setting up controller1
        controller1 = new Controller(gamepad1, 0.0f, "1",
                new Controller.Key[] {
                        Controller.Key.LEFT_STICK_X,
                        Controller.Key.LEFT_STICK_Y,
                        Controller.Key.RIGHT_STICK_X,
                        Controller.Key.RIGHT_STICK_Y
                });

        //Setting up controller2
        controller2 = new Controller(gamepad2, 0.0f, "2",
                new Controller.Key[] {
                        Controller.Key.LEFT_STICK_X,
                        Controller.Key.LEFT_STICK_Y,
                        Controller.Key.RIGHT_STICK_X,
                        Controller.Key.RIGHT_STICK_Y
                });

        LogController.logInfo("Waiting for start...");
        waitForStart();
        LogController.logInfo("Starting TeleOp.");
        //What the robot does while running
        while (opModeIsActive()) {
            LogController.logData();
            //Updates all active PID loops
            PIDController.update();

            //Uses the left joystick and right joystick horizontally to drive the robot with mecanumDrive and drives slower with the left bumper pressed
            if (controller1.getButton(Controller.Key.BUMPER_LEFT)) {
                drive.mecanumDrive(0.25 * controller1.analogDeadband(Controller.Key.RIGHT_STICK_X), -0.25 * controller1.analogDeadband(Controller.Key.LEFT_STICK_Y), -0.25 * controller1.analogDeadband(Controller.Key.LEFT_STICK_X));
            }
            else {
                drive.mecanumDrive(0.88 * controller1.analogDeadband(Controller.Key.RIGHT_STICK_X), -0.75 * controller1.analogDeadband(Controller.Key.LEFT_STICK_Y), -0.75 * controller1.analogDeadband(Controller.Key.LEFT_STICK_X));
            }
            //The x and y say if you move thhe joystick horizontally or vertically

            //Moves intake while left stick is being pushed vertically on controller2
            nonDriveMotors.get("Intake").setPower(-1 * controller2.getAnalog(Controller.Key.LEFT_STICK_Y));
        }

        //Closes all logs
        LogController.logInfo("TeleOp complete.");
        LogController.closeLogs();
    }
}
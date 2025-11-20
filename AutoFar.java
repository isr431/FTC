package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoFar extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    // Timer
    private ElapsedTime runtime = new ElapsedTime();
    
    // CONFIGURATION PARAMETERS
    private static final double INITIAL_PAUSE = 2.0;  // Seconds to wait before moving
    private static final double DRIVE_TIME = 2.0;     // Seconds to drive forward
    private static final double DRIVE_POWER = 0.5;    // Motor power (0.0 to 1.0)
    
    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        
        // Set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Display initialization status
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Initial Pause", "%.1f seconds", INITIAL_PAUSE);
        telemetry.addData("Drive Time", "%.1f seconds", DRIVE_TIME);
        telemetry.addData("Drive Power", "%.1f", DRIVE_POWER);
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        // STEP 1: Initial pause
        telemetry.addData("Status", "Pausing...");
        telemetry.update();
        sleep((long)(INITIAL_PAUSE * 1000));  // Convert seconds to milliseconds
        
        // STEP 2: Drive forward
        telemetry.addData("Status", "Driving Forward");
        telemetry.update();
        
        // Set all motors to drive forward
        frontLeft.setPower(DRIVE_POWER);
        frontRight.setPower(DRIVE_POWER);
        backLeft.setPower(DRIVE_POWER);
        backRight.setPower(DRIVE_POWER);
        
        // Wait for the specified drive time
        sleep((long)(DRIVE_TIME * 1000));
        
        // STEP 3: Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        
        telemetry.addData("Status", "Complete");
        telemetry.addData("Total Time", "%.1f seconds", runtime.seconds());
        telemetry.update();
    }
}
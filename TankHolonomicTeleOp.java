package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Tank Holonomic TeleOp", group="TeleOp")
public class TankHolonomicTeleOp extends OpMode {
    
    // ========== DRIVETRAIN MOTORS ==========
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    // ========== DRIVETRAIN SPEED CONSTANTS ==========
    // Master speed multiplier (affects all motors equally)
    private static final double MAX_DRIVE_SPEED = 1.0; // Set between 0.0 and 1.0
    
    // Individual motor speed multipliers (for fine-tuning if motors have different strengths)
    private static final double FRONT_LEFT_MULTIPLIER = 1.0;
    private static final double FRONT_RIGHT_MULTIPLIER = 1.0;
    private static final double BACK_LEFT_MULTIPLIER = 1.0;
    private static final double BACK_RIGHT_MULTIPLIER = 1.0;
    
    // ========== OTHER MOTORS ==========
    private DcMotor catapultLeft;
    private DcMotor catapultRight;
    private DcMotor auxMotor;
    
    // ========== CATAPULT CONSTANTS ==========
    private static final double CATAPULT_WIND_SPEED = 0.8;
    private static final double CATAPULT_RELEASE_SPEED = 1.0;
    private static final int CATAPULT_RELEASE_DEGREES = 360;
    private static final double DEGREES_TO_TICKS = 28 * 40 / 360.0;
    
    private boolean isReleasing = false;
    private int catapultTargetPosition = 0;
    
    // ========== AUX MOTOR CONSTANTS ==========
    private static final double AUX_MOTOR_SPEED = 1.0;
    
    @Override
    public void init() {
        // Initialize drivetrain motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        
        // Initialize catapult motors
        catapultLeft = hardwareMap.get(DcMotor.class, "catapultLeft");
        catapultRight = hardwareMap.get(DcMotor.class, "catapultRight");
        
        // Initialize auxiliary motor
        auxMotor = hardwareMap.get(DcMotor.class, "auxMotor");
        
        // Set drivetrain motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set catapult motor directions (opposite to each other)
        catapultLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        catapultRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set auxiliary motor direction
        auxMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Set zero power behavior for drivetrain
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Set zero power behavior for other motors
        catapultLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapultRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        auxMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reset catapult encoders
        catapultLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapultRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapultLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        catapultRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // ========== DRIVETRAIN CONTROL ==========
        controlDrivetrain();
        
        // ========== CATAPULT CONTROL ==========
        controlCatapult();
        
        // ========== AUXILIARY MOTOR CONTROL ==========
        controlAuxMotor();
        
        // ========== TELEMETRY ==========
        updateTelemetry();
    }
    
    private void controlDrivetrain() {
        // Get gamepad inputs
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightY = -gamepad1.right_stick_y;
        double rightX = gamepad1.right_stick_x;
        
        // Calculate motion components
        double drive = (leftY + rightY) / 2.0;   // Forward/backward
        double strafe = (leftX + rightX) / 2.0;  // Left/right strafe
        double rotate = (rightY - leftY) / 2.0;  // Rotation
        
        // Calculate raw power for each wheel using holonomic formulas
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;
        
        // Normalize wheel powers (keep within -1.0 to 1.0)
        double maxPower = Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );
        
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        
        // Apply master speed and individual motor multipliers
        frontLeftPower *= MAX_DRIVE_SPEED * FRONT_LEFT_MULTIPLIER;
        frontRightPower *= MAX_DRIVE_SPEED * FRONT_RIGHT_MULTIPLIER;
        backLeftPower *= MAX_DRIVE_SPEED * BACK_LEFT_MULTIPLIER;
        backRightPower *= MAX_DRIVE_SPEED * BACK_RIGHT_MULTIPLIER;
        
        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
    
    private void controlCatapult() {
        // Right trigger: Wind catapult backwards
        if (gamepad1.right_trigger > 0.1) {
            if (isReleasing) {
                catapultLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                catapultRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                isReleasing = false;
            }
            
            catapultLeft.setPower(CATAPULT_WIND_SPEED * gamepad1.right_trigger);
            catapultRight.setPower(CATAPULT_WIND_SPEED * gamepad1.right_trigger);
        }
        // Left trigger: Release catapult
        else if (gamepad1.left_trigger > 0.1 && !isReleasing) {
            isReleasing = true;
            
            int currentPos = catapultLeft.getCurrentPosition();
            catapultTargetPosition = currentPos - (int)(CATAPULT_RELEASE_DEGREES * DEGREES_TO_TICKS);
            
            catapultLeft.setTargetPosition(catapultTargetPosition);
            catapultRight.setTargetPosition(catapultTargetPosition);
            
            catapultLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            catapultRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            catapultLeft.setPower(CATAPULT_RELEASE_SPEED);
            catapultRight.setPower(CATAPULT_RELEASE_SPEED);
        }
        // Check if release is complete
        else if (isReleasing) {
            if (!catapultLeft.isBusy() && !catapultRight.isBusy()) {
                catapultLeft.setPower(0);
                catapultRight.setPower(0);
                catapultLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                catapultRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                isReleasing = false;
            }
        }
        else {
            catapultLeft.setPower(0);
            catapultRight.setPower(0);
        }
    }
    
    private void controlAuxMotor() {
        if (gamepad1.b) {
            auxMotor.setPower(AUX_MOTOR_SPEED);
        } else if (gamepad1.x) {
            auxMotor.setPower(-AUX_MOTOR_SPEED);
        } else {
            auxMotor.setPower(0);
        }
    }
    
    private void updateTelemetry() {
        telemetry.addData("Status", "Running");
        telemetry.addData("", "");
        
        // Drivetrain info
        telemetry.addData("FL Power", "%.2f", frontLeft.getPower());
        telemetry.addData("FR Power", "%.2f", frontRight.getPower());
        telemetry.addData("BL Power", "%.2f", backLeft.getPower());
        telemetry.addData("BR Power", "%.2f", backRight.getPower());
        telemetry.addData("", "");
        
        // Catapult info
        telemetry.addData("Catapult Status", isReleasing ? "RELEASING" : "Ready");
        telemetry.addData("Catapult Position", catapultLeft.getCurrentPosition());
        telemetry.addData("", "");
        
        // Auxiliary motor info
        telemetry.addData("Aux Motor Power", "%.2f", auxMotor.getPower());
        
        telemetry.update();
    }
}
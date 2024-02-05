package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;


@Autonomous(name="AutonomoAzulDireito" , group="Linear Opmode")
public class AutonomoAzulDireito extends LinearOpMode {
    static final double DRIVE_SPEED = 0.6;
    static final double TIME_OUT = 15.0;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorEf;
    private DcMotor motorEt;
    private DcMotor motorDf;
    private DcMotor motorDt;

    class Linear {
        public Linear() {

        }
    }

    class Chassi {
        DcMotor[] motors;
        private final HashMap<String, DcMotor.Direction> directionEnumMap = new HashMap<String, DcMotor.Direction>() {{
            put("forward", DcMotor.Direction.FORWARD);
            put("reverse", DcMotor.Direction.REVERSE);
        }};

        public Chassi(DcMotor[] motors) {
            this.motors = motors;
        }

        private void applyChassiMotorDirections(
                int distance, String... directions) {
            String directionKeys[] = directions;
            for (int i = 0; i < this.motors.length; i++) {
                DcMotor motor = this.motors[i];
                DcMotor.Direction direction = this.directionEnumMap.get(directionKeys[i]);
                setupMotor(motor, direction);
            }
            encoderDrive(DRIVE_SPEED, distance, TIME_OUT);
        }

        public void moveForward(int distance) {
            //                    {motorEf, motorDf, motorEt, motorDt}
            String[] directions = {"reverse", "forward", "reverse", "forward"};
            this.applyChassiMotorDirections(distance, directions[0], directions[1], directions[2], directions[3]);
        }

        public void moveBackward(int distance) {
            String[] directions = {"forward", "reverse", "forward", "reverse"};
            this.applyChassiMotorDirections(distance, directions[0], directions[1], directions[2], directions[3]);
        }

        public void moveLeft(int distance) {
            //                    {motorEf, motorDf, motorEt, motorDt}
            String[] directions = {"forward", "forward", "reverse", "reverse"};
            this.applyChassiMotorDirections(distance, directions[0], directions[1], directions[2], directions[3]);
        }

        public void moveRight(int distance) {
            String[] directions = {"reverse", "reverse", "forward", "forward"};
            this.applyChassiMotorDirections(distance, directions[0], directions[1], directions[2], directions[3]);
        }

        public void turnLeft(int angle) {
            setupMotor(this.motors[0], DcMotor.Direction.FORWARD);  // motorEf
            setupMotor(this.motors[1], DcMotor.Direction.FORWARD);  // motorEt
            setupMotor(this.motors[2], DcMotor.Direction.FORWARD);  // motorDf
            setupMotor(this.motors[3], DcMotor.Direction.FORWARD);  // motorDt
            // Calcular quanto as rodas têm que se mover para que o Robô atinja certo ângulo
            motorMove(this.motors[0], angle);  // motorEf
            motorMove(this.motors[1], angle);  // motorEt
            motorMove(this.motors[2], angle);  // motorDf
            motorMove(this.motors[3], angle);  // motorDt
        }

        public void turnRight(int angle) {
            setupMotor(this.motors[0], DcMotor.Direction.REVERSE);  // motorEf
            setupMotor(this.motors[1], DcMotor.Direction.REVERSE);  // motorEt
            setupMotor(this.motors[2], DcMotor.Direction.REVERSE);  // motorDf
            setupMotor(this.motors[3], DcMotor.Direction.REVERSE);  // motorDt
            // Calcular quanto as rodas têm que se mover para que o Robô atinja certo ângulo
            motorMove(this.motors[0], angle);  // motorEf
            motorMove(this.motors[1], angle);  // motorEt
            motorMove(this.motors[2], angle);  // motorDf
            motorMove(this.motors[3], angle);  // motorDt
        }
    }

    class TeamRobot {
        Chassi chassi;
        Linear linear;

        public TeamRobot(Chassi chassi, Linear linear) {
            this.chassi = chassi;
            this.linear = linear;
        }

        public void move(String direction, float distance) {
            int length = (int) Math.floor(distance);
            switch (direction) {
                case "forward":
                    this.chassi.moveForward(length);
                    break;
                case "backward":
                    this.chassi.moveBackward(length);
                    break;
                case "left":
                    this.chassi.moveLeft(length);
                    break;
                case "right":
                    this.chassi.moveRight(length);
                    break;
                default:
                    break;
            }
        }

        public void turn(String direction, int angle) {
            switch (direction) {
                case "left":
                    this.chassi.turnLeft(angle);
                    break;
                case "right":
                    this.chassi.turnRight(angle);
                    break;
                default:
                    break;
            }
        }


    }

    static final double COUNTS_PER_MOTOR_REV = 28.0;
    static final double DRIVE_GEAR_REDUCTION = 30.4;
    static final double WHEEL_CIRCUMFERENCE_MM = 90.0 * 3.14;
    static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double COUNTS_PER_MM = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;
    static final int TATAMI_SIDE_SIZE = 584;

    public static void setupMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void motorMove(DcMotor motor, int distance) {
        int target = (int) (distance * COUNTS_PER_MM);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.4);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorEf = hardwareMap.get(DcMotor.class, "ef");  //PORTA 0 EXPANSION HUB
        motorEt = hardwareMap.get(DcMotor.class, "et");  //PORTA 3 EXPANSION HUB
        motorDf = hardwareMap.get(DcMotor.class, "df");  //PORTA 3 CONTROL HUB
        motorDt = hardwareMap.get(DcMotor.class, "dt");  //PORTA 0 CONTROL HUB

        DcMotor[] tractionMotors = {motorEf, motorDf, motorEt, motorDt};

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {

            Linear linear = new Linear();
            Chassi chassi = new Chassi(tractionMotors);
            TeamRobot robot = new TeamRobot(chassi, linear);

            //andar até passagem
            //Lado do Tatame = 584

            robot.move("forward", 584);  //mover 1/4 do tatame
            robot.move("backward", 500);  //mover 1/4 do tatame
            robot.move("right", TATAMI_SIDE_SIZE * 2);  //mover 1/4 do tatame
//            robot.move("backward", 292);  //mover 1/4 do tatame
//            robot.move("right", 292);  //mover 1/4 do tatame


            while (opModeIsActive() && (motorEf.isBusy() && motorDt.isBusy() && motorDf.isBusy() && motorEt.isBusy())) {
                telemetry.addData("motorDf:", motorDf.getCurrentPosition());
                telemetry.addData("motorDt:", motorDt.getCurrentPosition());
                telemetry.addData("motorEf:", motorEf.getCurrentPosition());
                telemetry.addData("motorEt:", motorEt.getCurrentPosition());
                telemetry.update();
            }

//            motorEf.setPower(0);
//            motorEt.setPower(0);
//            motorDf.setPower(0);
//            motorDt.setPower(0);
//            sleep(1000);
        }
    }

    public void encoderDrive(double speed,
                             double distanceMm, //, double rightInches,
                             double timeoutS) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int newLeftFrontTarget = motorEf.getCurrentPosition() + (int) (distanceMm * COUNTS_PER_MM);
            int newLeftBackTarget = motorEt.getCurrentPosition() + (int) (distanceMm * COUNTS_PER_MM);
            int newRightFrontTarget = motorDf.getCurrentPosition() + (int) (distanceMm * COUNTS_PER_MM);
            int newRightBackTarget = motorDt.getCurrentPosition() + (int) (distanceMm * COUNTS_PER_MM);
            motorEf.setTargetPosition(newLeftFrontTarget);
            motorEt.setTargetPosition(newLeftBackTarget);
            motorDf.setTargetPosition(newRightFrontTarget);
            motorDt.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            motorEf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorEt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorEf.setPower(Math.abs(speed));
            motorEt.setPower(Math.abs(speed));
            motorDf.setPower(Math.abs(speed));
            motorDt.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorEf.isBusy() && motorEt.isBusy() && motorDf.isBusy() && motorDt.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftFrontTarget,
                        newLeftBackTarget,
                        newRightFrontTarget,
                        newRightBackTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        motorEf.getCurrentPosition(), motorEt.getCurrentPosition(),
                        motorDf.getCurrentPosition(), motorDt.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorEf.setPower(0);
            motorEt.setPower(0);
            motorDf.setPower(0);
            motorDt.setPower(0);

            // Turn off RUN_TO_POSITION
            motorEf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorEt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            sleep(250);   // optional pause after each move.
        }
    }
}

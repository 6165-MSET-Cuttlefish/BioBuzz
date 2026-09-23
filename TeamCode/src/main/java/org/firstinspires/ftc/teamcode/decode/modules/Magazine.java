package org.firstinspires.ftc.teamcode.decode.modules;

import static org.firstinspires.ftc.teamcode.decode.DecodeRobot.magazineTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.architecture.core.Module;
import org.firstinspires.ftc.teamcode.architecture.core.State;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedMotor;
import org.firstinspires.ftc.teamcode.architecture.hardware.EnhancedServo;
import org.firstinspires.ftc.teamcode.architecture.prism.Color;
import org.firstinspires.ftc.teamcode.architecture.prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.architecture.prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.decode.DecodeContext;

@Config("Decode Magazine")
public class Magazine extends Module {
    private final EnhancedServo horizontalFront;
    private final EnhancedServo horizontalBack;
    private final EnhancedMotor intake;
    private final EnhancedMotor vertical;
    private final EnhancedServo headlightFront;
    private final EnhancedServo headlightMiddle;
    private final EnhancedServo headlightBack;
    private final NormalizedColorSensor frontLeftColor, frontRightColor;
    private final NormalizedColorSensor middleFrontColor, middleBackColor;
    private final NormalizedColorSensor backRightColor, backLeftColor;
    private final DistanceSensor frontLeftDistance;
    private final DistanceSensor frontRightDistance;
    private final DistanceSensor middleFrontDistance;
    private final DistanceSensor middleBackDistance;
    private final DistanceSensor backLeftDistance;
    private final DistanceSensor backRightDistance;
    public GoBildaPrismDriver prism;

    private Endgame endgame;

    public static PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.BLUE);
    public static PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();
    public static boolean prismWritesEnabled = true;

    public static boolean optimizeServoCachingTolerances = true;
    public static boolean optimizeSensorCadence = false;
    public static int optimizeDistanceSensorEveryNLoops = 2;
    public static int optimizeColorSensorEveryNLoops = 2;

    private static final int FRONT_START_LEFT = 0;
    private static final int FRONT_END_LEFT   = 5;
    private static final int FRONT_START_RIGHT = 30;
    private static final int FRONT_END_RIGHT   = 35;

    private static final int MIDDLE_START_LEFT = 6;
    private static final int MIDDLE_END_LEFT   = 11;
    private static final int MIDDLE_START_RIGHT = 24;
    private static final int MIDDLE_END_RIGHT   = 29;

    private static final int BACK_START_LEFT = 12;
    private static final int BACK_END_LEFT   = 17;
    private static final int BACK_START_RIGHT = 18;
    private static final int BACK_END_RIGHT   = 23;

    private final PrismAnimations.Solid frontLeftSlotSolid   = new PrismAnimations.Solid();
    private final PrismAnimations.Solid frontRightSlotSolid  = new PrismAnimations.Solid();
    private final PrismAnimations.Solid middleLeftSlotSolid  = new PrismAnimations.Solid();
    private final PrismAnimations.Solid middleRightSlotSolid = new PrismAnimations.Solid();
    private final PrismAnimations.Solid backLeftSlotSolid    = new PrismAnimations.Solid();
    private final PrismAnimations.Solid backRightSlotSolid   = new PrismAnimations.Solid();
    private Color currentStatusPrismColor = null;
    private boolean currentStatusPrismIsSnake = false;
    private boolean prismFlashActive = false;
    public double horizontalFrontPosition, horizontalBackPosition;
    public double intakePower, verticalPower;
    public double headlightFrontPosition;
    public double headlightMiddlePosition;
    public double headlightBackPosition;

    private MagazineState currentState = new MagazineState(
            MagazineState.ArtifactColor.EMPTY,
            MagazineState.ArtifactColor.EMPTY,
            MagazineState.ArtifactColor.EMPTY);

    private double strobePosition = 0.28;
    private boolean strobeIncreasing = true;
    private long lastStrobeUpdate = 0;
    private static final double STROBE_MIN = 0.28;
    private static final double STROBE_MAX = 0.71;
    private static final double STROBE_SPEED = 0.05;
    private static final long STROBE_INTERVAL_MS = 50;

    private boolean backSlotsFilled = false;
    private boolean middleSlotFilled = false;
    private int backSlotsFilledLoops = 0;
    private int middleSlotFilledLoops = 0;
    private int frontSlotFilledLoops = 0;
    private long intakeBallAbsentStartTime = 0;
    public boolean intakeIsFull = false;
    public boolean shotSorted = true;
    public static int slotFillConsecutiveLoops = 3;
    public static long intakeBallDropoutGraceMs = 200;

    // mm; a ball is present when the reading is below the threshold.
    public static double frontLeftDistanceThreshold = 45;
    public static double frontRightDistanceThreshold = 30;

    public static double middleBackDistanceThreshold = 40;
    public static double middleFrontDistanceThreshold = 20;

    public static double backLeftDistanceThreshold = 40;
    public static double backRightDistanceThreshold = 40;

    public static int horizontalTime = 400;
    public static boolean updateDistanceSensor = true;
    public static boolean updateColorSensor = true;

    private int distanceSensorLoopCounter = 0;
    private int colorSensorLoopCounter = 0;

    private MagazineState.ArtifactColor savedBackColor = MagazineState.ArtifactColor.EMPTY;
    private MagazineState.ArtifactColor savedMiddleColor = MagazineState.ArtifactColor.EMPTY;
    private MagazineState.ArtifactColor savedFrontColor = MagazineState.ArtifactColor.EMPTY;

    // Servo position units per millisecond.
    public static double horizontalFrontSpeed = 0.0008;
    public static double horizontalBackSpeed = 0.0005;
    private double currentHorizontalFrontPosition = Double.NaN;
    private double currentHorizontalBackPosition = Double.NaN;
    private long lastHorizontalFrontUpdateTime = 0;
    private long lastHorizontalBackUpdateTime = 0;

    public enum HorizontalFrontState implements State {
        OPEN(0.6),
        OPEN_SHOOT(OPEN.getValue() - 0.125),
        STORED(0.06),
        MANUAL(-1);

        HorizontalFrontState(double value) {
            setValue(value);
        }
    }

    public enum HorizontalBackState implements State {
        OPEN(0.39),
        OPEN_SHOOT(OPEN.getValue() - 0.02),
        STORED(0.68),
        STORED_SHOOT(STORED.getValue() - 0.02);

        HorizontalBackState(double value) {
            setValue(value);
        }
    }

    public enum IntakeState implements State {
        FORWARD(1),
        SHOOTING(1),
        HALF(0.5),
        IDLE(0.3),
        REVERSE(-1),
        OFF(0),
        ANALOG_EXTAKE(0),
        AUTO_EXTAKE(-0.5);

        IntakeState(double value) {
            setValue(value);
        }
    }

    public enum VerticalState implements State {
        ON(1),
        HALF_DOWN(-0.5),
        OFF(0);

        VerticalState(double value) {
            setValue(value);
        }
    }

    private static final double HEADLIGHT_OFF = 0;
    private static final double HEADLIGHT_RED = 0.28;
    private static final double HEADLIGHT_STROBE_SENTINEL = -1;
    private static final double HEADLIGHT_ORANGE = 0.333;
    private static final double HEADLIGHT_GREEN = .5;
    private static final double HEADLIGHT_CYAN_STROBE = 0.6;
    private static final double HEADLIGHT_YELLOW = 0.39;
    private static final double HEADLIGHT_WHITE = 0.8;
    private static final double HEADLIGHT_PURPLE = 0.722;
    private static final double HEADLIGHT_WHITE_STROBE = 1;

    private static final double HEADLIGHT_FRONT_CYAN = 0.518;
    private static final double HEADLIGHT_MIDDLE_CYAN = 0.522;
    private static final double HEADLIGHT_BACK_CYAN = 0.526;

    public enum HeadlightFrontState implements State {
        OFF(HEADLIGHT_OFF), RED(HEADLIGHT_RED), RED_STROBE(HEADLIGHT_RED),
        RED_GREEN_STROBE(HEADLIGHT_STROBE_SENTINEL), BLUE_GREEN_STROBE(HEADLIGHT_STROBE_SENTINEL),
        ORANGE(HEADLIGHT_ORANGE), GREEN(HEADLIGHT_GREEN), CYAN(HEADLIGHT_FRONT_CYAN),
        CYAN_STROBE(HEADLIGHT_CYAN_STROBE), YELLOW(HEADLIGHT_YELLOW),
        WHITE(HEADLIGHT_WHITE), PURPLE(HEADLIGHT_PURPLE), WHITE_STROBE(HEADLIGHT_WHITE_STROBE),
        STROBE(HEADLIGHT_STROBE_SENTINEL);
        HeadlightFrontState(double value) { setValue(value); }
    }

    public enum HeadlightMiddleState implements State {
        OFF(HEADLIGHT_OFF), RED(HEADLIGHT_RED), RED_STROBE(HEADLIGHT_RED),
        RED_GREEN_STROBE(HEADLIGHT_STROBE_SENTINEL), BLUE_GREEN_STROBE(HEADLIGHT_STROBE_SENTINEL),
        ORANGE(HEADLIGHT_ORANGE), GREEN(HEADLIGHT_GREEN), CYAN(HEADLIGHT_MIDDLE_CYAN),
        CYAN_STROBE(HEADLIGHT_CYAN_STROBE), YELLOW(HEADLIGHT_YELLOW),
        WHITE(HEADLIGHT_WHITE), PURPLE(HEADLIGHT_PURPLE), WHITE_STROBE(HEADLIGHT_WHITE_STROBE),
        STROBE(HEADLIGHT_STROBE_SENTINEL);
        HeadlightMiddleState(double value) { setValue(value); }
    }

    public enum HeadlightBackState implements State {
        OFF(HEADLIGHT_OFF), RED(HEADLIGHT_RED), RED_STROBE(HEADLIGHT_RED),
        RED_GREEN_STROBE(HEADLIGHT_STROBE_SENTINEL), BLUE_GREEN_STROBE(HEADLIGHT_STROBE_SENTINEL),
        ORANGE(HEADLIGHT_ORANGE), GREEN(HEADLIGHT_GREEN), CYAN(HEADLIGHT_BACK_CYAN),
        CYAN_STROBE(HEADLIGHT_CYAN_STROBE), YELLOW(HEADLIGHT_YELLOW),
        WHITE(HEADLIGHT_WHITE), PURPLE(HEADLIGHT_PURPLE), WHITE_STROBE(HEADLIGHT_WHITE_STROBE),
        STROBE(HEADLIGHT_STROBE_SENTINEL);
        HeadlightBackState(double value) { setValue(value); }
    }

    public enum PrismState implements State {
        OFF(0),
        BLUE_BLINK(1),
        RAINBOW(2);

        PrismState(double value) {
            setValue(value);
        }
    }

    public Magazine(HardwareMap hardwareMap) {
        super();
        setTelemetryEnabled(magazineTelemetry.TOGGLE);

        double servoTol = optimizeServoCachingTolerances ? 0.005 : 0.0;
        double headlightTol = optimizeServoCachingTolerances ? 0.003 : 0.0;
        horizontalFront = new EnhancedServo(hardwareMap, "horizontalFront").withCachingTolerance(servoTol);
        horizontalBack = new EnhancedServo(hardwareMap, "horizontalBack").withCachingTolerance(servoTol);
        intake = new EnhancedMotor(hardwareMap, "intake");
        vertical = new EnhancedMotor(hardwareMap, "vertical");
        headlightFront = new EnhancedServo(hardwareMap, "headlightFront").withCachingTolerance(headlightTol);
        headlightMiddle = new EnhancedServo(hardwareMap, "headlightMiddle").withCachingTolerance(headlightTol);
        headlightBack = new EnhancedServo(hardwareMap, "headlightBack").withCachingTolerance(headlightTol);

        frontLeftColor = hardwareMap.get(NormalizedColorSensor.class, "frontLeftColor");
        frontRightColor = hardwareMap.get(NormalizedColorSensor.class, "frontRightColor");
        middleFrontColor = hardwareMap.get(NormalizedColorSensor.class, "middleFrontColor");
        middleBackColor = hardwareMap.get(NormalizedColorSensor.class, "middleBackColor");
        backRightColor = hardwareMap.get(NormalizedColorSensor.class, "backRightColor");
        backLeftColor = hardwareMap.get(NormalizedColorSensor.class, "backLeftColor");
        frontLeftDistance = (DistanceSensor) frontLeftColor;
        frontRightDistance = (DistanceSensor) frontRightColor;
        middleFrontDistance = (DistanceSensor) middleFrontColor;
        middleBackDistance = (DistanceSensor) middleBackColor;
        backLeftDistance = (DistanceSensor) backLeftColor;
        backRightDistance = (DistanceSensor) backRightColor;

        vertical.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.setStripLength(36);
        prism.setTargetFPS(30);

        onEnter(PrismState.OFF, () ->
                loadPrismArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2));
        onEnter(PrismState.BLUE_BLINK, () ->
                loadPrismArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0));
        onEnter(PrismState.RAINBOW, () ->
                loadPrismArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1));

        onEnter(VerticalState.ON, this::resetBallDetection);
        onEnter(IntakeState.ANALOG_EXTAKE, this::resetBallDetection);

        // onEnter never fires for the initial state, so clear the Prism explicitly here.
        loadPrismArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
    }

    /** Required: write() PWM-disables the front and middle headlights while the endgame disables servos. */
    public Magazine withEndgame(Endgame endgame) {
        this.endgame = endgame;
        return this;
    }

    private Endgame requireEndgame() {
        if (endgame == null) {
            throw new IllegalStateException("Magazine needs the Endgame; pass it with withEndgame()");
        }
        return endgame;
    }

    @Override
    protected void initStates() {
        setStates(HorizontalFrontState.OPEN, HorizontalBackState.OPEN,
                IntakeState.OFF, VerticalState.OFF,
                HeadlightFrontState.ORANGE, HeadlightMiddleState.ORANGE, HeadlightBackState.ORANGE,
                PrismState.OFF);
    }

    @Override
    protected void read() {
        double targetHorizontalFrontPos = getState(HorizontalFrontState.class).getValue();
        double targetHorizontalBackPos = getState(HorizontalBackState.class).getValue();

        if (Double.isNaN(currentHorizontalFrontPosition)) {
            currentHorizontalFrontPosition = targetHorizontalFrontPos;
        }
        if (Double.isNaN(currentHorizontalBackPosition)) {
            currentHorizontalBackPosition = targetHorizontalBackPos;
        }

        if (targetHorizontalFrontPos >= 0) {
            long now = System.currentTimeMillis();
            if (lastHorizontalFrontUpdateTime == 0) lastHorizontalFrontUpdateTime = now;
            long dtMs = now - lastHorizontalFrontUpdateTime;
            lastHorizontalFrontUpdateTime = now;

            double maxStep = horizontalFrontSpeed * dtMs;
            if (currentHorizontalFrontPosition < targetHorizontalFrontPos) {
                currentHorizontalFrontPosition = Math.min(currentHorizontalFrontPosition + maxStep, targetHorizontalFrontPos);
            } else if (currentHorizontalFrontPosition > targetHorizontalFrontPos) {
                currentHorizontalFrontPosition = Math.max(currentHorizontalFrontPosition - maxStep, targetHorizontalFrontPos);
            }
        }

        if (targetHorizontalBackPos >= 0) {
            long now = System.currentTimeMillis();
            if (lastHorizontalBackUpdateTime == 0) lastHorizontalBackUpdateTime = now;
            long dtMs = now - lastHorizontalBackUpdateTime;
            lastHorizontalBackUpdateTime = now;

            double maxStep = horizontalBackSpeed * dtMs;
            if (currentHorizontalBackPosition < targetHorizontalBackPos) {
                currentHorizontalBackPosition = Math.min(currentHorizontalBackPosition + maxStep, targetHorizontalBackPos);
            } else if (currentHorizontalBackPosition > targetHorizontalBackPos) {
                currentHorizontalBackPosition = Math.max(currentHorizontalBackPosition - maxStep, targetHorizontalBackPos);
            }
        }

        horizontalFrontPosition = currentHorizontalFrontPosition;
        horizontalBackPosition = currentHorizontalBackPosition;

        intakePower = getState(IntakeState.class).getValue();
        verticalPower = getState(VerticalState.class).getValue();

        HeadlightFrontState frontState = getState(HeadlightFrontState.class);
        if (frontState == HeadlightFrontState.STROBE) {
            updateColorStrobe();
        } else if (frontState == HeadlightFrontState.RED_GREEN_STROBE || frontState == HeadlightFrontState.BLUE_GREEN_STROBE) {
            updateTwoColorStrobe();
        } else if (frontState == HeadlightFrontState.CYAN_STROBE || frontState == HeadlightFrontState.WHITE_STROBE || frontState == HeadlightFrontState.RED_STROBE) {
            updateBlinkStrobe();
        } else {
            headlightFrontPosition = frontState.getValue();
        }

        headlightMiddlePosition = getState(HeadlightMiddleState.class).getValue();
        headlightBackPosition   = getState(HeadlightBackState.class).getValue();

        if (getState(VerticalState.class) == VerticalState.ON
                || getState(IntakeState.class) == IntakeState.ANALOG_EXTAKE) {
            resetBallDetection();
        }

        if (updateDistanceSensor) {
            if (shouldUpdateDistanceSensorThisLoop()) {
                updateDistanceDetection();
            }
        }

        if (updateColorSensor) {
            if (shouldUpdateColorSensorThisLoop()) {
                updateMagazineColorState();
            }
        }
    }

    @Override
    protected void write() {
        horizontalFront.setPosition(horizontalFrontPosition);
        horizontalBack.setPosition(horizontalBackPosition);
        intake.setPower(intakePower);
        vertical.setPower(verticalPower);
        if (!requireEndgame().disableServosForEndgame) {
            headlightFront.setPosition(headlightFrontPosition);
            headlightMiddle.setPosition(headlightMiddlePosition);
        } else {
            headlightFront.setPwmDisable();
            headlightMiddle.setPwmDisable();
        }
        headlightBack.setPosition(headlightBackPosition);
    }

    @Override
    public void stop() {
        intake.setPower(0);
        vertical.setPower(0);
    }

    public void updateMagazineColorState() {
        if (!backSlotsFilled) {
            clearBackAndForwardColorLatches();
            currentState = new MagazineState(
                    MagazineState.ArtifactColor.EMPTY,
                    MagazineState.ArtifactColor.EMPTY,
                    MagazineState.ArtifactColor.EMPTY);
            return;
        }

        // detectColor's position runs front=1..back=3, the reverse of MagazineState's; thresholds are tuned to it.
        if (savedBackColor == MagazineState.ArtifactColor.EMPTY) {
            savedBackColor = detectSectionColor(backRightColor, backLeftColor, 3);
        }

        if (!middleSlotFilled) {
            clearMiddleAndFrontColorLatches();
            currentState = new MagazineState(
                    savedBackColor,
                    MagazineState.ArtifactColor.EMPTY,
                    MagazineState.ArtifactColor.EMPTY);
            return;
        }

        if (savedMiddleColor == MagazineState.ArtifactColor.EMPTY) {
            savedMiddleColor = detectSectionColor(middleFrontColor, middleBackColor, 2);
        }

        if (!intakeIsFull) {
            clearFrontColorLatch();
            currentState = new MagazineState(
                    savedBackColor,
                    savedMiddleColor,
                    MagazineState.ArtifactColor.EMPTY);
            return;
        }

        if (savedFrontColor == MagazineState.ArtifactColor.EMPTY) {
            savedFrontColor = detectSectionColor(frontLeftColor, frontRightColor, 1);
        }

        currentState = new MagazineState(savedBackColor, savedMiddleColor, savedFrontColor);
    }

    private void clearBackAndForwardColorLatches() {
        savedBackColor = MagazineState.ArtifactColor.EMPTY;
        clearMiddleAndFrontColorLatches();
    }

    private void clearMiddleAndFrontColorLatches() {
        savedMiddleColor = MagazineState.ArtifactColor.EMPTY;
        clearFrontColorLatch();
    }

    private void clearFrontColorLatch() {
        savedFrontColor = MagazineState.ArtifactColor.EMPTY;
    }

    private void updateDistanceDetection() {
        if (!backSlotsFilled) {
            double backLeftDist = backLeftDistance.getDistance(DistanceUnit.MM);
            double backRightDist = backRightDistance.getDistance(DistanceUnit.MM);
            boolean backPairFilled = backLeftDist < backLeftDistanceThreshold
                    || backRightDist < backRightDistanceThreshold;

            if (backPairFilled) {
                backSlotsFilledLoops++;
                if (backSlotsFilledLoops >= slotFillConsecutiveLoops) {
                    backSlotsFilled = true;
                    backSlotsFilledLoops = 0;
                }
            } else {
                backSlotsFilledLoops = 0;
            }

            middleSlotFilledLoops = 0;
            frontSlotFilledLoops = 0;
            intakeIsFull = false;
            return;
        }

        if (!middleSlotFilled) {
            double middleFrontDist = middleFrontDistance.getDistance(DistanceUnit.MM);
            double middleBackDist = middleBackDistance.getDistance(DistanceUnit.MM);
            boolean middleFilledNow = middleFrontDist < middleFrontDistanceThreshold
                    || middleBackDist < middleBackDistanceThreshold;

            if (middleFilledNow) {
                middleSlotFilledLoops++;
                if (middleSlotFilledLoops >= slotFillConsecutiveLoops) {
                    middleSlotFilled = true;
                    middleSlotFilledLoops = 0;
                }
            } else {
                middleSlotFilledLoops = 0;
            }

            frontSlotFilledLoops = 0;
            intakeIsFull = false;
            return;
        }

        double frontLeftDist = frontLeftDistance.getDistance(DistanceUnit.MM);
        double frontRightDist = frontRightDistance.getDistance(DistanceUnit.MM);
        boolean frontFilledNow = frontLeftDist < frontLeftDistanceThreshold
                && frontRightDist < frontRightDistanceThreshold;

        if (frontFilledNow) {
            frontSlotFilledLoops++;
            if (frontSlotFilledLoops >= slotFillConsecutiveLoops) {
                intakeIsFull = true;
                intakeBallAbsentStartTime = 0;
            }
        } else {
            frontSlotFilledLoops = 0;
            if (intakeIsFull) {
                long now = System.currentTimeMillis();
                if (intakeBallAbsentStartTime == 0) {
                    intakeBallAbsentStartTime = now;
                }
                if ((now - intakeBallAbsentStartTime) >= intakeBallDropoutGraceMs) {
                    intakeIsFull = false;
                    intakeBallAbsentStartTime = 0;
                }
            }
        }
    }

    private void resetBallDetection() {
        backSlotsFilled = false;
        middleSlotFilled = false;
        backSlotsFilledLoops = 0;
        middleSlotFilledLoops = 0;
        frontSlotFilledLoops = 0;
        intakeIsFull = false;
        intakeBallAbsentStartTime = 0;
        clearBackAndForwardColorLatches();
        currentState = new MagazineState(
                MagazineState.ArtifactColor.EMPTY,
                MagazineState.ArtifactColor.EMPTY,
                MagazineState.ArtifactColor.EMPTY);
    }

    private boolean shouldUpdateDistanceSensorThisLoop() {
        if (!optimizeSensorCadence) {
            return true;
        }
        int every = Math.max(1, optimizeDistanceSensorEveryNLoops);
        return (distanceSensorLoopCounter++ % every) == 0;
    }

    private boolean shouldUpdateColorSensorThisLoop() {
        if (!optimizeSensorCadence) {
            return true;
        }
        int every = Math.max(1, optimizeColorSensorEveryNLoops);
        return (colorSensorLoopCounter++ % every) == 0;
    }

    public void updateMagazinePrismLeds() {
        if (!isPrismWriteAllowed()) return;

        Color frontColor = getPrismColorForArtifact(currentState.getPosition3());
        Color middleColor = getPrismColorForArtifact(currentState.getPosition2());
        Color backColor = getPrismColorForArtifact(currentState.getPosition1());

        frontLeftSlotSolid.setBrightness(100);
        frontLeftSlotSolid.setPrimaryColor(frontColor);
        frontLeftSlotSolid.setIndexes(FRONT_START_LEFT, FRONT_END_LEFT);
        insertAndUpdatePrismAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, frontLeftSlotSolid);

        frontRightSlotSolid.setBrightness(100);
        frontRightSlotSolid.setPrimaryColor(frontColor);
        frontRightSlotSolid.setIndexes(FRONT_START_RIGHT, FRONT_END_RIGHT);
        insertAndUpdatePrismAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, frontRightSlotSolid);

        middleLeftSlotSolid.setBrightness(100);
        middleLeftSlotSolid.setPrimaryColor(middleColor);
        middleLeftSlotSolid.setIndexes(MIDDLE_START_LEFT, MIDDLE_END_LEFT);
        insertAndUpdatePrismAnimation(GoBildaPrismDriver.LayerHeight.LAYER_2, middleLeftSlotSolid);

        middleRightSlotSolid.setBrightness(100);
        middleRightSlotSolid.setPrimaryColor(middleColor);
        middleRightSlotSolid.setIndexes(MIDDLE_START_RIGHT, MIDDLE_END_RIGHT);
        insertAndUpdatePrismAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, middleRightSlotSolid);

        backLeftSlotSolid.setBrightness(100);
        backLeftSlotSolid.setPrimaryColor(backColor);
        backLeftSlotSolid.setIndexes(BACK_START_LEFT, BACK_END_LEFT);
        insertAndUpdatePrismAnimation(GoBildaPrismDriver.LayerHeight.LAYER_4, backLeftSlotSolid);

        backRightSlotSolid.setBrightness(100);
        backRightSlotSolid.setPrimaryColor(backColor);
        backRightSlotSolid.setIndexes(BACK_START_RIGHT, BACK_END_RIGHT);
        insertAndUpdatePrismAnimation(GoBildaPrismDriver.LayerHeight.LAYER_5, backRightSlotSolid);
    }

    private Color getPrismColorForArtifact(MagazineState.ArtifactColor artifactColor) {
        switch (artifactColor) {
            case GREEN:
                return Color.GREEN;
            case PURPLE:
                return Color.PURPLE;
            case EMPTY:
            default:
                return Color.WHITE;
        }
    }

    private MagazineState.ArtifactColor detectSectionColor(
            NormalizedColorSensor sensor1, NormalizedColorSensor sensor2, int position) {
        MagazineState.ArtifactColor color1 = detectColor(sensor1, position);
        MagazineState.ArtifactColor color2 = detectColor(sensor2, position);
        if (color1 != MagazineState.ArtifactColor.EMPTY) return color1;
        return color2;
    }

    private MagazineState.ArtifactColor detectColor(NormalizedColorSensor sensor, int position) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        float red = rgba.red * 255;
        float green = rgba.green * 255;
        float blue = rgba.blue * 255;

        float total = red + green + blue;
        if (total == 0) return MagazineState.ArtifactColor.EMPTY;

        double redRatio = red / total * 100;
        double greenRatio = green / total * 100;
        double blueRatio = blue / total * 100;

        int greenScore = 0;
        int purpleScore = 0;

        double greenRedMax   = position == 1 ? 20.0 : (position == 2 ? 20.0 : 21.0);
        double greenGreenMin  = position == 1 ? 44.0 : (position == 2 ? 47.0 : 45.0);
        double purpleBlueMin   = position == 1 ? 40.0 : (position == 2 ? 36.0 : 37.0);
        double purpleGreenMax = position == 1 ? 35.0 : (position == 2 ? 37.0 : 37.0);

        if (redRatio < greenRedMax)    greenScore++;
        if (greenRatio > greenGreenMin)  greenScore++;
        if (blueRatio > purpleBlueMin)     purpleScore++;
        if (greenRatio < purpleGreenMax) purpleScore++;

        if (greenScore > purpleScore && greenScore > 1) return MagazineState.ArtifactColor.GREEN;
        if (purpleScore > greenScore && purpleScore > 1) return MagazineState.ArtifactColor.PURPLE;
        return MagazineState.ArtifactColor.EMPTY;
    }

    public String getColorPattern() {
        return currentState.toPattern();
    }

    public MagazineState.ArtifactColor getSensorColor(int sensorNumber) {
        return currentState.getPosition(sensorNumber);
    }

    public int getBallCount() {
        int balls = 0;
        for (int i = 1; i <= 3; i++) {
            if (getSensorColor(i) != MagazineState.ArtifactColor.EMPTY) {
                balls++;
            }
        }
        return balls;
    }

    public MagazineState getMagazineState() {
        return currentState;
    }

    public boolean isFull() {
        return currentState.isFull();
    }

    public boolean isBackSlotFilled() {
        return backSlotsFilled;
    }

    public boolean isMiddleSlotFilled() {
        return middleSlotFilled;
    }

    public void setStatusPrismColor(Color color) {
        if (!isPrismWriteAllowed() || color == null) return;
        if (color == currentStatusPrismColor && !currentStatusPrismIsSnake) return;

        GoBildaPrismDriver.Artboard artboard;
        if (color == Color.RED) {
            artboard = GoBildaPrismDriver.Artboard.ARTBOARD_4;
        } else if (color == Color.WHITE) {
            artboard = GoBildaPrismDriver.Artboard.ARTBOARD_5;
        } else if (color == Color.PURPLE) {
            artboard = GoBildaPrismDriver.Artboard.ARTBOARD_6;
        } else {
            return;
        }

        currentStatusPrismColor = color;
        currentStatusPrismIsSnake = false;
        loadPrismArtboard(artboard);
    }

    public void setStatusPrismSnake() {
        if (!isPrismWriteAllowed() || currentStatusPrismIsSnake) return;

        currentStatusPrismColor = null;
        currentStatusPrismIsSnake = true;
        loadPrismArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3);
    }

    public void setPrismFlash(boolean enabled) {
        if (!isPrismWriteAllowed()) {
            prismFlashActive = false;
            return;
        }

        if (enabled) {
            if (!prismFlashActive) {
                PrismState.BLUE_BLINK.activate();
                prismFlashActive = true;
            }
            currentStatusPrismColor = null;
            currentStatusPrismIsSnake = false;
        } else {
            prismFlashActive = false;
        }
    }

    public boolean isEmpty() {
        return currentState.isEmpty();
    }

    public int countColor(MagazineState.ArtifactColor color) {
        return currentState.countColor(color);
    }

    /** horizontalBack stores the back-slot ball; horizontalFront stores the middle-slot ball. */
    public StorageDecision checkAndStoreBalls(MagazineState target) {
        updateMagazineColorState();

        MagazineState.ArtifactColor ball1 = currentState.getPosition1();
        MagazineState.ArtifactColor ball2 = currentState.getPosition2();
        MagazineState.ArtifactColor ball3 = currentState.getPosition3();

        MagazineState.ArtifactColor color1 = target.getPosition1();
        MagazineState.ArtifactColor color2 = target.getPosition2();

        if (countColor(MagazineState.ArtifactColor.GREEN) >= 2
                ||  countColor(MagazineState.ArtifactColor.PURPLE) == 3
                || getBallCount() <= 2) {
            return new StorageDecision(false, false);
        }

        boolean storeInBack;
        boolean storeInFront;

        if (ball1 == color1) {
            storeInBack = false;
            MagazineState.ArtifactColor color3 = target.getPosition3();
            storeInFront = !(ball2 == color2 && ball3 == color3);
        } else {
            storeInBack = true;
            // Storing the back ball shifts ball2 to back and ball3 to middle, hence the index skew.
            storeInFront = !(ball2 == color1 && ball3 == color2);
        }

        return new StorageDecision(storeInBack, storeInFront);
    }

    public static class StorageDecision {
        public final boolean storeInBack;
        public final boolean storeInFront;

        public StorageDecision(boolean storeInBack, boolean storeInFront) {
            this.storeInBack = storeInBack;
            this.storeInFront = storeInFront;
        }

        @Override
        public String toString() {
            return String.format("StorageDecision[Back=%s, Front=%s]", storeInBack, storeInFront);
        }
    }

    private void updateColorStrobe() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastStrobeUpdate >= STROBE_INTERVAL_MS) {
            lastStrobeUpdate = currentTime;
            if (strobeIncreasing) {
                strobePosition += STROBE_SPEED;
                if (strobePosition >= STROBE_MAX) {
                    strobePosition = STROBE_MAX;
                    strobeIncreasing = false;
                }
            } else {
                strobePosition -= STROBE_SPEED;
                if (strobePosition <= STROBE_MIN) {
                    strobePosition = STROBE_MIN;
                    strobeIncreasing = true;
                }
            }
            headlightFrontPosition = strobePosition;
        }
    }

    private void updateBlinkStrobe() {
        long currentTime = System.currentTimeMillis();
        if (Math.sin((double) currentTime / 1000 * 2 * Math.PI) > 0) {
            headlightFrontPosition = HeadlightFrontState.OFF.getValue();
        } else {
            headlightFrontPosition = getState(HeadlightFrontState.class).getValue();
        }
    }

    private void updateTwoColorStrobe() {
        long currentTime = System.currentTimeMillis();
        HeadlightFrontState frontState = getState(HeadlightFrontState.class);
        boolean firstColor = (currentTime / 500) % 2 == 0;
        if (frontState == HeadlightFrontState.RED_GREEN_STROBE) {
            headlightFrontPosition = firstColor
                    ? HeadlightFrontState.RED.getValue()
                    : HeadlightFrontState.GREEN.getValue();
        } else if (frontState == HeadlightFrontState.BLUE_GREEN_STROBE) {
            headlightFrontPosition = firstColor
                    ? HeadlightFrontState.CYAN.getValue()
                    : HeadlightFrontState.GREEN.getValue();
        }
    }

    public double getIntakeCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    private boolean isPrismWriteAllowed() {
        return prismWritesEnabled && prism != null;
    }

    private void loadPrismArtboard(GoBildaPrismDriver.Artboard artboard) {
        if (!isPrismWriteAllowed()) return;
        prism.loadAnimationsFromArtboard(artboard);
    }

    private void insertAndUpdatePrismAnimation(GoBildaPrismDriver.LayerHeight layer,
                                               PrismAnimations.AnimationBase animation) {
        if (!isPrismWriteAllowed()) return;
        prism.insertAndUpdateAnimation(layer, animation);
    }

    public double getVerticalCurrent() {
        return vertical.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    protected void onTelemetry() {
        if (magazineTelemetry.TOGGLE) {
            if (magazineTelemetry.intake) {
                logDashboard("Intake State", getState(IntakeState.class));
                logDashboard("Intake Power", "%.3f", intakePower);
            }
            if (magazineTelemetry.vertical) {
                logDashboard("Vertical State", getState(VerticalState.class));
                logDashboard("Vertical Power", "%.3f", verticalPower);
            }
            if (magazineTelemetry.servos) {
                logDashboard("Horizontal Front State", getState(HorizontalFrontState.class));
                logDashboard("Horizontal Front Position", "%.3f", horizontalFrontPosition);
                logDashboard("Horizontal Back State", getState(HorizontalBackState.class));
                logDashboard("Horizontal Back Position", "%.3f", horizontalBackPosition);
            }
            if (magazineTelemetry.headlights) {
                logDashboard("Headlight Front", getState(HeadlightFrontState.class));
                logDashboard("Headlight Middle", getState(HeadlightMiddleState.class));
                logDashboard("Headlight Back", getState(HeadlightBackState.class));
            }
            if (magazineTelemetry.current) {
                logDashboard("Intake Current (A)", "%.2f", intake.getCurrent(CurrentUnit.AMPS));
                logDashboard("Vertical Current (A)", "%.2f", vertical.getCurrent(CurrentUnit.AMPS));
            }
            if (magazineTelemetry.colorSensors) {
                log("Current Pattern", currentState.toPattern());
                log("Motif", DecodeContext.motif.toPattern());
                logDashboard("Back", currentState.getPosition1());
                logDashboard("Middle", currentState.getPosition2());
                logDashboard("Front", currentState.getPosition3());
                logDashboard("Front Ball Detected", intakeIsFull);
            }
        }
    }
}

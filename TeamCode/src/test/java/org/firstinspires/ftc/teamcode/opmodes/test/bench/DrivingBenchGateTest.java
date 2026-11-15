package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.Test;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.xml.parsers.DocumentBuilderFactory;

public class DrivingBenchGateTest {

    @Test
    public void aBenchThatDrivesRefusesTheRobotsConfig() {
        try {
            BenchRobot.drivingFollower(robotMap());
            fail("expected the driving follower to refuse a config without benchServo");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("\"benchServo\""));
            assertTrue(e.getMessage(), e.getMessage().contains("run_bench.py"));
        }
    }

    interface Bench {
        void build(HardwareMap hardwareMap) throws Exception;
    }

    @Test
    public void everyBenchThatPowersTheDriveMotorsRefusesAConfigWithoutBenchServo() throws Exception {
        Map<String, Bench> benches = new LinkedHashMap<>();
        benches.put("BenchRobot's follower", map -> unconstructed(BenchRobot.class).createFollower(map));
        benches.put("Planner Stop's camera robot's follower",
                map -> unconstructed(PlannerStopBench.CameraRobot.class).createFollower(map));
        benches.put("Limelight Fault's robot's follower",
                map -> unconstructed(LimelightFaultBench.FaultRobot.class).createFollower(map));
        benches.put("Raw Loop", map -> {
            RawLoopBench bench = new RawLoopBench();
            bench.hardwareMap = map;
            bench.runOpMode();
        });
        benches.put("Power Readback", map -> {
            PowerReadbackBench bench = new PowerReadbackBench();
            bench.hardwareMap = map;
            bench.init();
        });
        benches.put("PinpointSetup.create", PinpointSetup::create);
        benches.put("OctoQuadSetup.create", OctoQuadSetup::create);
        benches.put("Bench: Pinpoint's robot's follower",
                map -> unconstructed(PinpointBench.PinpointRobot.class).createFollower(map));
        benches.put("Bench: Pinpoint's soak robot's follower",
                map -> unconstructed(PinpointBench.SoakRobot.class).createFollower(map));
        benches.put("Bench: Pinpoint's no-odometry robot's follower",
                map -> unconstructed(PinpointBench.NoOdometryRobot.class).createFollower(map));
        benches.put("Bench: OctoQuad's robot's follower",
                map -> unconstructed(OctoQuadSetup.OctoQuadRobot.class).createFollower(map));
        benches.put("Bench: Pinpoint, any mode", map -> {
            PinpointBench bench = unconstructed(PinpointBench.class);
            bench.hardwareMap = map;
            bench.createRobot();
        });
        benches.put("Bench: OctoQuad, any mode", map -> {
            OctoQuadBench bench = unconstructed(OctoQuadBench.class);
            bench.hardwareMap = map;
            bench.createRobot();
        });

        List<String> wrong = new ArrayList<>();
        for (Map.Entry<String, Bench> bench : benches.entrySet()) {
            List<String> motorCalls = new ArrayList<>();
            try {
                bench.getValue().build(robotMap(motorCalls));
                wrong.add(bench.getKey() + " started");
            } catch (IllegalStateException e) {
                if (!e.getMessage().contains("\"benchServo\"")) wrong.add(bench.getKey() + ": " + e.getMessage());
            } catch (Throwable e) {
                wrong.add(bench.getKey() + " threw " + e);
            }
            if (!motorCalls.isEmpty()) wrong.add(bench.getKey() + " touched " + motorCalls);
        }
        assertTrue(wrong.toString(), wrong.isEmpty());
    }

    @Test
    public void aFollowerOnARealLocalizerRefusesTheRobotsConfigBeforeBuildingTheLocalizer() {
        List<String> built = new ArrayList<>();
        try {
            BenchRobot.benchFollower(robotMap(), map -> {
                built.add("localizer");
                return new SoftwareLocalizer();
            });
            fail("expected the bench follower to refuse a config without benchServo");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("\"benchServo\""));
        }
        assertTrue(built.toString(), built.isEmpty());
    }

    private static final String BENCH_SOURCES = "src/main/java/org/firstinspires/ftc/teamcode/opmodes/test/bench";
    private static final Pattern FOLLOWER_BUILD = Pattern.compile(
            "new Follower\\(|new Mecanum\\(|new CappedMecanum\\(|BioBuzzConstants\\.(?:create|drivetrain)\\(");
    private static final Pattern PEDRO_DRIVES = Pattern.compile(
            "\\.(?:follow|hold|manual)\\(|\\b(?:PathCommands|PedroCommands|RouteRun|PathActionBuilder)\\b");

    @Test
    public void everyBenchFollowerComesFromBenchRobotAndNoBenchLetsPedroDriveAnUngatedOne() throws Exception {
        File[] sources = new File(BENCH_SOURCES).listFiles((dir, name) -> name.endsWith(".java"));
        assertTrue("no bench sources under " + BENCH_SOURCES, sources != null && sources.length > 0);
        List<String> wrong = new ArrayList<>();
        for (File source : sources) {
            String text = new String(Files.readAllBytes(source.toPath()), StandardCharsets.UTF_8);
            String name = source.getName();
            if (!name.equals("BenchRobot.java") && FOLLOWER_BUILD.matcher(text).find()) {
                wrong.add(name + " builds its own follower or drivetrain");
            }
            if (PEDRO_DRIVES.matcher(text).find() && text.contains("softwareFollower(")) {
                wrong.add(name + " lets Pedro drive and takes BenchRobot.softwareFollower, which isn't gated");
            }
        }
        String benchRobot = new String(Files.readAllBytes(new File(BENCH_SOURCES, "BenchRobot.java").toPath()),
                StandardCharsets.UTF_8);
        Matcher build = Pattern.compile("new Follower\\([^;]*;").matcher(benchRobot);
        int builds = 0;
        while (build.find()) {
            builds++;
            if (!build.group().contains("cappedDrivetrain(")) wrong.add("BenchRobot builds an uncapped follower: " + build.group());
        }
        assertTrue("BenchRobot builds no follower", builds > 0);
        assertTrue(wrong.toString(), wrong.isEmpty());
    }

    private static <T> T unconstructed(Class<T> type) throws Exception {
        Class<?> unsafeClass = Class.forName("sun.misc.Unsafe");
        Field theUnsafe = unsafeClass.getDeclaredField("theUnsafe");
        theUnsafe.setAccessible(true);
        return type.cast(unsafeClass.getMethod("allocateInstance", Class.class).invoke(theUnsafe.get(null), type));
    }

    @Test
    public void theRunnersConfigPassesTheGate() {
        BenchRobot.requireBenchConfig(robotMap().with("benchServo", fake(Servo.class)));
    }

    @Test
    public void aConfigWithBenchServoThatNamesTheRobotsIntakeOrTransferIsRefused() {
        for (String robotMotor : new String[] {"intake", "transfer"}) {
            try {
                BenchRobot.requireBenchConfig(robotMap().with("benchServo", fake(Servo.class))
                        .with(robotMotor, fake(DcMotorEx.class)));
                fail("expected a config naming " + robotMotor + " to be refused");
            } catch (IllegalStateException e) {
                assertTrue(e.getMessage(), e.getMessage().contains("\"" + robotMotor + "\""));
                assertTrue(e.getMessage(), e.getMessage().contains("run_bench.py"));
            }
        }
    }

    @Test
    public void theRobotsOwnConfigWithABenchServoAddedIsRefused() throws Exception {
        Element robot = DocumentBuilderFactory.newInstance().newDocumentBuilder()
                .parse(new File("src/main/res/xml/biobuzz.xml")).getDocumentElement();
        NodeList all = robot.getElementsByTagName("*");
        FakeHardwareMap map = new FakeHardwareMap().with("benchServo", fake(Servo.class));
        for (int i = 0; i < all.getLength(); i++) {
            Element device = (Element) all.item(i);
            if (!device.getTagName().startsWith("Lynx")) map.with(device.getAttribute("name"), fake(DcMotorEx.class));
        }
        for (String robotMotor : BenchRobot.ROBOT_ONLY_DEVICES) {
            assertTrue(robotMotor + " is no longer in biobuzz.xml", map.devices.containsKey(robotMotor));
        }
        try {
            BenchRobot.requireBenchConfig(map);
            fail("expected biobuzz.xml with a benchServo to be refused");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("the robot's"));
        }
    }

    private static FakeHardwareMap robotMap() {
        return robotMap(new ArrayList<>());
    }

    private static FakeHardwareMap robotMap(List<String> motorCalls) {
        FakeHardwareMap map = new FakeHardwareMap();
        for (String motor : new String[] {"fl", "fr", "bl", "br"}) map.with(motor, fake(DcMotorEx.class, motorCalls));
        return map;
    }

    private static <T extends HardwareDevice> T fake(Class<T> type) {
        return fake(type, new ArrayList<>());
    }

    @SuppressWarnings("unchecked")
    private static <T extends HardwareDevice> T fake(Class<T> type, List<String> calls) {
        return (T) Proxy.newProxyInstance(DrivingBenchGateTest.class.getClassLoader(), new Class<?>[] {type},
                (proxy, method, args) -> {
                    if (!method.getDeclaringClass().equals(Object.class)) calls.add(method.getName());
                    Class<?> r = method.getReturnType();
                    if (method.getName().equals("hashCode")) return System.identityHashCode(proxy);
                    if (method.getName().equals("equals")) return proxy == args[0];
                    if (r == boolean.class) return false;
                    if (r == int.class) return 0;
                    if (r == double.class) return 0.0;
                    return null;
                });
    }

    private static final class FakeHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();

        FakeHardwareMap() {
            super(null, null);
        }

        FakeHardwareMap with(String name, HardwareDevice device) {
            devices.put(name, device);
            return this;
        }

        @Override
        public SortedSet<String> getAllNames(Class<? extends HardwareDevice> classOrInterface) {
            SortedSet<String> names = new TreeSet<>();
            for (Map.Entry<String, HardwareDevice> e : devices.entrySet()) {
                if (classOrInterface.isInstance(e.getValue())) names.add(e.getKey());
            }
            return names;
        }

        @Override
        public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
            HardwareDevice device = devices.get(deviceName);
            if (!classOrInterface.isInstance(device)) {
                throw new IllegalArgumentException("no " + classOrInterface.getSimpleName() + " named " + deviceName);
            }
            return classOrInterface.cast(device);
        }
    }
}

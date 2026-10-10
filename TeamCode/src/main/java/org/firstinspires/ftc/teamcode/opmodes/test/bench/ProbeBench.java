package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import android.content.res.Resources;

import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * What is plugged into the hub, so run_bench.py can write a config that matches: the embedded IMU's chip, the
 * attached UVC webcams' serial numbers, and any Limelight on an Ethernet-over-USB interface.
 */
@TeleOp(name = "Bench: Probe", group = "Test")
public class ProbeBench extends LinearOpMode {

    @Override
    public void runOpMode() {
        BenchReport report = new BenchReport("Probe", "probe");
        while (opModeInInit()) {
            report.sendIfDue();
            sleep(20);
        }
        if (!opModeIsActive()) return;

        report.phase("hubs");
        List<Map<String, Object>> hubs = new ArrayList<>();
        for (LynxModule m : hardwareMap.getAll(LynxModule.class)) {
            Map<String, Object> h = new LinkedHashMap<>();
            h.put("names", new ArrayList<>(hardwareMap.getNamesOf(m)));
            h.put("parent", m.isParent());
            h.put("address", m.getModuleAddress());
            h.put("serial", m.getSerialNumber().getString());
            // name(), not toString(): toString() gives "BHI260AP" for BHI260.
            h.put("imuType", m.getImuType().name());
            h.put("firmware", m.getNullableFirmwareVersionString());
            h.put("responding", HubRegisters.responding(m));
            hubs.add(h);
        }
        report.put("hubs", hubs);

        report.phase("webcams");
        List<Map<String, Object>> webcams = new ArrayList<>();
        for (WebcamName w : ClassFactory.getInstance().getCameraManager().getAllWebcams()) {
            Map<String, Object> c = new LinkedHashMap<>();
            c.put("serial", w.getSerialNumber().getString());
            c.put("display", w.getSerialNumber().toString());
            c.put("usbDevice", w.getUsbDeviceNameIfAttached());
            c.put("attached", w.isAttached());
            webcams.add(c);
        }
        report.put("webcams", webcams);

        report.phase("ethernet");
        List<Map<String, Object>> limelights = new ArrayList<>();
        report.put("interfaces", interfaces(limelights));
        report.put("limelights", limelights);

        report.phase("configs");
        RobotConfigFileManager configs = new RobotConfigFileManager();
        Resources resources = hardwareMap.appContext.getResources();
        RobotConfigFile active = configs.getActiveConfig();
        Map<String, Object> activeConfig = new LinkedHashMap<>();
        activeConfig.put("name", active.getName());
        activeConfig.put("location", active.getLocation().name());
        activeConfig.put("resourceId", active.getResourceId());
        if (active.getLocation() == RobotConfigFile.FileLocation.RESOURCE) {
            activeConfig.put("resolvesTo", resourceName(resources, active.getResourceId()));
        }
        report.put("activeConfig", activeConfig);
        Map<String, Object> shipped = new LinkedHashMap<>();
        for (RobotConfigFile f : configs.getXMLFiles()) {
            if (f.getLocation() == RobotConfigFile.FileLocation.RESOURCE) shipped.put(f.getName(), f.getResourceId());
        }
        report.put("shippedConfigs", shipped);
        int originalResourceId = (int) BenchIO.param(report.params, "originalResourceId", 0);
        if (originalResourceId != 0) report.put("originalResolvesTo", resourceName(resources, originalResourceId));

        report.done(hubs.size() + " hub(s), " + webcams.size() + " webcam(s), " + limelights.size() + " Limelight(s)");
        while (opModeIsActive()) {
            report.sendIfDue();
            sleep(20);
        }
    }

    /** Mirrors the SDK's scan: an eth* interface's IPv4 address names the device; the Limelight is that subnet's .1. */
    private static List<String> interfaces(List<Map<String, Object>> limelights) {
        List<String> all = new ArrayList<>();
        try {
            for (NetworkInterface ni : Collections.list(NetworkInterface.getNetworkInterfaces())) {
                for (InetAddress a : Collections.list(ni.getInetAddresses())) {
                    all.add(ni.getName() + " " + a.getHostAddress());
                    if (!ni.getName().startsWith("eth") || !(a instanceof Inet4Address)) continue;
                    byte[] b = a.getAddress();
                    String limelightIp = (b[0] & 0xff) + "." + (b[1] & 0xff) + "." + (b[2] & 0xff) + ".1";
                    Map<String, Object> l = new LinkedHashMap<>();
                    l.put("serial", "EthernetOverUsb:" + ni.getName() + ":" + a.getHostAddress());
                    l.put("ipAddress", limelightIp);
                    l.put("status", httpGet("http://" + limelightIp + ":5807/status"));
                    limelights.add(l);
                }
            }
        } catch (SocketException e) {
            throw new IllegalStateException("can't list network interfaces", e);
        }
        return all;
    }

    private static String resourceName(Resources resources, int id) {
        try {
            return resources.getResourceTypeName(id) + "/" + resources.getResourceEntryName(id);
        } catch (Resources.NotFoundException e) {
            return "missing";
        }
    }

    private static String httpGet(String url) {
        HttpURLConnection c = null;
        try {
            c = (HttpURLConnection) new URL(url).openConnection();
            c.setConnectTimeout(1000);
            c.setReadTimeout(1000);
            if (c.getResponseCode() != 200) return "HTTP " + c.getResponseCode();
            StringBuilder sb = new StringBuilder();
            try (BufferedReader r = new BufferedReader(new InputStreamReader(c.getInputStream(), StandardCharsets.UTF_8))) {
                String line;
                while ((line = r.readLine()) != null) sb.append(line);
            }
            return sb.toString();
        } catch (IOException e) {
            // Probe result, not a failure: a Limelight that is still booting doesn't answer yet.
            return "unreachable: " + e;
        } finally {
            if (c != null) c.disconnect();
        }
    }
}

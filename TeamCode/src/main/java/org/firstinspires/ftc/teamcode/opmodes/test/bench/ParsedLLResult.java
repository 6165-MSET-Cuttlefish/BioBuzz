package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.json.JSONException;
import org.json.JSONObject;

/** LLResult's parser is protected; this is the same parse Limelight3A's poller runs on every result. */
final class ParsedLLResult extends LLResult {
    ParsedLLResult(JSONObject json) throws JSONException {
        super(json);
    }

    static LLResult parse(String json) {
        try {
            return new ParsedLLResult(new JSONObject(json));
        } catch (JSONException e) {
            throw new IllegalArgumentException("not a Limelight result: " + e.getMessage(), e);
        }
    }

    /** A Python-pipeline result shaped like a Limelight 3A's /results reply, with {@code balls} ball-script detections. */
    static String synthetic(int balls, double tsMs) {
        return synthetic(balls, 0, 0, tsMs);
    }

    /**
     * As {@link #synthetic(int, double)} plus {@code fiducials} AprilTag and {@code detectors} neural-detector entries
     * carrying every field LLResultTypes parses, so LLResult's per-entry work scales as a busy pipeline's would.
     */
    static String synthetic(int balls, int fiducials, int detectors, double tsMs) {
        StringBuilder py = new StringBuilder();
        py.append("6166,").append(balls);
        for (int i = 0; i < balls; i++) py.append(",1,").append(20.5 + i * 3.25).append(',').append(-4.75 + i * 2.5);
        for (int i = 2 + 3 * balls; i < 32; i++) py.append(",0");
        String six = "[0,0,0,0,0,0]";
        StringBuilder fid = new StringBuilder();
        for (int i = 0; i < fiducials; i++) {
            if (i > 0) fid.append(',');
            String pose = "[" + (0.12 + i * 0.01) + ",-0.034,1.42,2.5,-11.25,178.5]";
            fid.append("{\"fID\":").append(20 + i).append(",\"fam\":\"36H11C\",").append(corners(i))
                    .append(",\"skew\":0.012,\"t6c_ts\":").append(pose).append(",\"t6r_fs\":").append(pose)
                    .append(",\"t6r_ts\":").append(pose).append(",\"t6t_cs\":").append(pose).append(",\"t6t_rs\":")
                    .append(pose).append(',').append(target(i)).append('}');
        }
        StringBuilder det = new StringBuilder();
        for (int i = 0; i < detectors; i++) {
            if (i > 0) det.append(',');
            det.append("{\"class\":\"pollen\",\"classID\":").append(i % 3).append(",\"conf\":").append(0.91 - i * 0.004)
                    .append(',').append(corners(i)).append(',').append(target(i)).append('}');
        }
        return "{\"Barcode\":[],\"Classifier\":[],\"Detector\":[" + det + "],\"Fiducial\":[" + fid + "],\"Retro\":[],"
                + "\"PythonOut\":[" + py + "],"
                + "\"botpose\":" + six + ",\"botpose_avgarea\":0,\"botpose_avgdist\":0,\"botpose_span\":0,\"botpose_tagcount\":0,"
                + "\"botpose_wpiblue\":" + six + ",\"botpose_wpired\":" + six + ",\"botpose_orb\":" + six
                + ",\"botpose_orb_wpiblue\":" + six + ",\"botpose_orb_wpired\":" + six
                + ",\"cl\":11.84,\"focus_metric\":0,\"pID\":4,\"pTYPE\":\"pipe_python\",\"pipelineType\":\"pipe_python\","
                + "\"stdev_mt1\":" + six + ",\"stdev_mt2\":" + six + ",\"t6c_rs\":" + six
                + ",\"tl\":6.21,\"ts\":" + tsMs + ",\"ts_nt\":" + (long) (tsMs * 1000) + ",\"ts_sys\":" + (long) (tsMs * 1000)
                + ",\"ts_us\":" + (long) (tsMs * 1000) + ",\"ta\":0,\"tx\":0,\"txnc\":0,\"ty\":0,\"tync\":0,\"v\":1}";
    }

    private static String corners(int i) {
        double x = 100 + i * 7.5;
        double y = 80 + i * 3.25;
        return "\"pts\":[[" + x + "," + y + "],[" + (x + 42.5) + "," + y + "],[" + (x + 42.5) + "," + (y + 40.25) + "],["
                + x + "," + (y + 40.25) + "]]";
    }

    private static String target(int i) {
        return "\"ta\":" + (0.8 + i * 0.01) + ",\"tx\":" + (-12.5 + i * 0.75) + ",\"tx_nocross\":" + (-12.5 + i * 0.75)
                + ",\"txp\":" + (121.25 + i * 7.5) + ",\"ty\":" + (4.25 - i * 0.5) + ",\"ty_nocross\":" + (4.25 - i * 0.5)
                + ",\"typ\":" + (100.5 + i * 3.25);
    }
}

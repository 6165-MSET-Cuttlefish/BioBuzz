package org.firstinspires.ftc.teamcode.opmodes.test.bench;

import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/** Writes Maps, Lists, arrays, numbers, booleans and strings as JSON; NaN and infinities become null. */
final class BenchJson {
    private BenchJson() {}

    static String write(Object value) {
        StringBuilder sb = new StringBuilder(4096);
        append(sb, value);
        return sb.toString();
    }

    private static void append(StringBuilder sb, Object v) {
        if (v == null) {
            sb.append("null");
        } else if (v instanceof String || v instanceof Character || v instanceof Enum) {
            quote(sb, v.toString());
        } else if (v instanceof Boolean) {
            sb.append(v.toString());
        } else if (v instanceof Double || v instanceof Float) {
            double d = ((Number) v).doubleValue();
            if (Double.isNaN(d) || Double.isInfinite(d)) sb.append("null");
            else sb.append(d);
        } else if (v instanceof Number) {
            sb.append(v.toString());
        } else if (v instanceof Map) {
            sb.append('{');
            boolean first = true;
            for (Map.Entry<?, ?> e : ((Map<?, ?>) v).entrySet()) {
                if (!first) sb.append(',');
                first = false;
                quote(sb, String.valueOf(e.getKey()));
                sb.append(':');
                append(sb, e.getValue());
            }
            sb.append('}');
        } else if (v instanceof Iterable) {
            sb.append('[');
            boolean first = true;
            for (Object o : (Iterable<?>) v) {
                if (!first) sb.append(',');
                first = false;
                append(sb, o);
            }
            sb.append(']');
        } else if (v instanceof double[]) {
            double[] a = (double[]) v;
            sb.append('[');
            for (int i = 0; i < a.length; i++) {
                if (i > 0) sb.append(',');
                append(sb, a[i]);
            }
            sb.append(']');
        } else if (v instanceof int[]) {
            int[] a = (int[]) v;
            sb.append('[');
            for (int i = 0; i < a.length; i++) {
                if (i > 0) sb.append(',');
                sb.append(a[i]);
            }
            sb.append(']');
        } else if (v instanceof long[]) {
            long[] a = (long[]) v;
            sb.append('[');
            for (int i = 0; i < a.length; i++) {
                if (i > 0) sb.append(',');
                sb.append(a[i]);
            }
            sb.append(']');
        } else if (v instanceof Object[]) {
            Object[] a = (Object[]) v;
            sb.append('[');
            for (int i = 0; i < a.length; i++) {
                if (i > 0) sb.append(',');
                append(sb, a[i]);
            }
            sb.append(']');
        } else {
            throw new IllegalArgumentException("BenchJson can't write a " + v.getClass().getName());
        }
    }

    private static void quote(StringBuilder sb, String s) {
        sb.append('"');
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"': sb.append("\\\""); break;
                case '\\': sb.append("\\\\"); break;
                case '\n': sb.append("\\n"); break;
                case '\r': sb.append("\\r"); break;
                case '\t': sb.append("\\t"); break;
                default:
                    if (c < 0x20) sb.append(String.format("\\u%04x", (int) c));
                    else sb.append(c);
            }
        }
        sb.append('"');
    }

    /** Parses a JSON object into nested LinkedHashMaps and Lists. */
    static Map<String, Object> parseObject(String json) {
        try {
            return toMap(new JSONObject(json));
        } catch (JSONException e) {
            throw new IllegalArgumentException("not a JSON object: " + e.getMessage(), e);
        }
    }

    private static Map<String, Object> toMap(JSONObject o) throws JSONException {
        Map<String, Object> m = new LinkedHashMap<>();
        Iterator<String> keys = o.keys();
        while (keys.hasNext()) {
            String k = keys.next();
            m.put(k, unwrap(o.get(k)));
        }
        return m;
    }

    private static Object unwrap(Object v) throws JSONException {
        if (v instanceof JSONObject) return toMap((JSONObject) v);
        if (v instanceof JSONArray) {
            JSONArray a = (JSONArray) v;
            List<Object> list = new java.util.ArrayList<>(a.length());
            for (int i = 0; i < a.length(); i++) list.add(unwrap(a.get(i)));
            return list;
        }
        if (v == JSONObject.NULL) return null;
        return v;
    }
}

package org.firstinspires.ftc.teamcode.architecture.telemetry;

public final class HtmlFormatter {
    private HtmlFormatter() {}

    public static final String COLOR_VALUE  = "#e37c07";
    public static final String COLOR_MODULE = "#4fc3f7";
    public static final String COLOR_STATE  = "#81c784";
    public static final String COLOR_GREEN  = "#66bb6a";
    public static final String COLOR_YELLOW = "#ffa726";
    public static final String COLOR_RED    = "#ef5350";
    public static final String COLOR_BLUE   = "#448aff";
    public static final String COLOR_GRAY   = "#9e9e9e";

    // Count of <big>/<small> wraps htmlSize applies, not a point size.
    public static final int FONT_SMALL   = -1;
    public static final int FONT_NORMAL  = 0;
    public static final int FONT_LARGE   = 1;
    public static final int FONT_XLARGE  = 2;
    public static final int FONT_XXLARGE = 3;

    public static String htmlBold(String text) {
        return "<b>" + text + "</b>";
    }

    /** The {@code html*()} primitives don't escape — callers must escape user-supplied content. */
    public static String htmlEscape(String text) {
        if (text == null) return "";
        if (text.indexOf('&') < 0 && text.indexOf('<') < 0
                && text.indexOf('>') < 0 && text.indexOf('"') < 0) {
            return text;
        }
        StringBuilder out = new StringBuilder(text.length() + 8);
        for (int i = 0; i < text.length(); i++) {
            char c = text.charAt(i);
            switch (c) {
                case '&': out.append("&amp;"); break;
                case '<': out.append("&lt;"); break;
                case '>': out.append("&gt;"); break;
                case '"': out.append("&quot;"); break;
                default: out.append(c);
            }
        }
        return out.toString();
    }

    public static String htmlColor(String hex, String text) {
        return "<font color='" + hex + "'>" + text + "</font>";
    }

    public static String htmlSize(int size, String text) {
        if (size == 0) return text;
        StringBuilder out = new StringBuilder(text.length() + 16);
        if (size > 0) {
            for (int i = 0; i < size; i++) out.append("<big>");
            out.append(text);
            for (int i = 0; i < size; i++) out.append("</big>");
        } else {
            int levels = -size;
            for (int i = 0; i < levels; i++) out.append("<small>");
            out.append(text);
            for (int i = 0; i < levels; i++) out.append("</small>");
        }
        return out.toString();
    }

    public static String htmlColorSize(String hex, int size, String text) {
        return htmlColor(hex, htmlSize(size, text));
    }
}

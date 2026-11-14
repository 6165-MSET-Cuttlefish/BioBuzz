package org.firstinspires.ftc.teamcode.architecture.telemetry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class HtmlFormatterTest {
    private static final String RED_OPEN = "<font color='" + HtmlFormatter.COLOR_FAULT + "'>";

    @Test
    public void faultLineIsBoldRedAroundAllOfItsText() {
        String line = HtmlFormatter.faultLine("Limelight", "no data for 0.3 s");
        assertTrue(line.startsWith("<b>" + RED_OPEN));
        assertTrue(line.endsWith("</font></b>"));
        String inside = line.substring(("<b>" + RED_OPEN).length(), line.length() - "</font></b>".length());
        assertFalse(inside.contains("<font"));
        assertTrue(inside.contains("FAULT Limelight"));
        assertTrue(inside.contains(": no data for 0.3 s"));
    }

    @Test
    public void keyIsExtraLargeAndMessageLarge() {
        String line = HtmlFormatter.faultLine("Webcam", "gone");
        assertTrue(line.contains("<big><big>FAULT Webcam</big></big>"));
        assertTrue(line.contains("<big>: gone</big>"));
        assertEquals("#ff1744", HtmlFormatter.COLOR_FAULT);
    }

    @Test
    public void keyAndMessageAreEscaped() {
        String line = HtmlFormatter.faultLine("a<b", "x > y & \"z\"");
        assertTrue(line.contains("FAULT a&lt;b"));
        assertTrue(line.contains(": x &gt; y &amp; &quot;z&quot;"));
        assertFalse(line.contains("a<b"));
    }

    @Test
    public void longMessageIsKeptWhole() {
        StringBuilder message = new StringBuilder();
        while (message.length() < 147) message.append('m');
        message.append("END");
        String line = HtmlFormatter.faultLine("Bench", message.toString());
        assertTrue(line.contains(message + "</big>"));
    }
}

package org.firstinspires.ftc.teamcode.architecture.telemetry;

import androidx.annotation.Nullable;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayDeque;
import java.util.Iterator;

import static org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter.*;
import static org.firstinspires.ftc.teamcode.architecture.OptimizationToggles.telemetryLazyFormat;

/**
 * Fan-out telemetry: structure (headers, separators, raw-HTML rows) goes to both screens as the same
 * HTML, data rows stay plain on the dashboard so its graph view keeps numbers, and the braille field
 * map is Driver Station only.
 */
public class DualTelemetry implements Telemetry {
    public static boolean enableDSTelemetry = true;
    public static boolean enableDashboardTelemetry = true;

    private final Telemetry dsTelemetry;
    private final Telemetry dashTelemetry;
    private TelemetryPacket packet;
    private boolean dsFormatApplied = false;
    private boolean dashFormatApplied = false;
    private final ArrayDeque<String> packetLog = new ArrayDeque<>();
    private int packetLogCapacity = 9;
    private Log.DisplayOrder packetLogOrder = Log.DisplayOrder.OLDEST_FIRST;
    private static final Item EMPTY_ITEM = new EnhancedItem(null, null);
    private static final String SEPARATOR_HTML =
            htmlColorSize(COLOR_GRAY, FONT_NORMAL, "─────────────────────────");

    public DualTelemetry(Telemetry dsTelemetry, Telemetry dashTelemetry) {
        this.dsTelemetry = dsTelemetry;
        this.dashTelemetry = dashTelemetry;
        ensureDisplayFormats();
    }

    // HTML mode is a one-shot setDisplayFormat() on each backend, so it must be re-applied on every
    // off→on toggle of the enable flags or raw HTML tags show up instead of markup. The dashboard
    // adapter additionally resets itself to CLASSIC at onOpModePreInit, which runs before the Robot
    // constructor that builds this; re-applying it any earlier would be undone.
    private void ensureDisplayFormats() {
        if (enableDSTelemetry && !dsFormatApplied) {
            dsTelemetry.setDisplayFormat(DisplayFormat.HTML);
            dsFormatApplied = true;
        } else if (!enableDSTelemetry) {
            dsFormatApplied = false;
        }
        if (enableDashboardTelemetry && !dashFormatApplied) {
            dashTelemetry.setDisplayFormat(DisplayFormat.HTML);
            dashFormatApplied = true;
        } else if (!enableDashboardTelemetry) {
            dashFormatApplied = false;
        }
    }

    public void setDSTransmissionInterval(int interval) {
        if (enableDSTelemetry) this.dsTelemetry.setMsTransmissionInterval(interval);
    }

    /**
     * Route dashboard data into {@code packet} (the same one carrying the field overlay) instead of
     * FtcDashboard's telemetry adapter. The adapter's {@code update()} sends a packet of its own whose
     * field overlay is empty; alternating that with the overlay packet is what makes the field view
     * flicker at loop rate. One packet per loop carrying both halves fixes it. Null restores the adapter.
     */
    public void setPacket(TelemetryPacket packet) { this.packet = packet; }

    private static boolean noSinkEnabled() {
        return telemetryLazyFormat && !enableDSTelemetry && !enableDashboardTelemetry;
    }

    private Item emit(String caption, Object value, boolean ds, boolean dash) {
        if (noSinkEnabled()) return EMPTY_ITEM;
        Item dsItem = ds && enableDSTelemetry
                ? dsTelemetry.addData(fmtCaption(caption), fmtValue(value))
                : null;
        Item dashItem = null;
        if (dash && enableDashboardTelemetry) {
            // Plain, never HTML: put() also feeds the keyed data the graph and CSV views read.
            if (packet != null) packet.put(caption, value);
            else dashItem = dashTelemetry.addData(caption, value);
        }
        return new EnhancedItem(dsItem, dashItem);
    }

    private Item emit(String caption, String format, Object[] args, boolean ds, boolean dash) {
        if (noSinkEnabled()) return EMPTY_ITEM;
        return emit(caption, String.format(format, args), ds, dash);
    }

    private void dashAddLine(String line) {
        if (!enableDashboardTelemetry) return;
        if (packet != null) packet.addLine(line);
        else dashTelemetry.addLine(line);
    }

    private String fmtCaption(String caption) {
        return htmlSize(FONT_NORMAL, htmlBold(htmlEscape(caption)));
    }

    private String fmtValue(Object value) {
        return htmlColorSize(COLOR_VALUE, FONT_NORMAL, htmlEscape(String.valueOf(value)));
    }

    public void addGroupHeader(String groupName) {
        addGroupHeader(groupName, COLOR_MODULE);
    }

    public void addGroupHeader(String groupName, String color) {
        String header = htmlBold(htmlColorSize(color, FONT_LARGE, htmlEscape(groupName)));
        if (enableDSTelemetry) dsTelemetry.addLine(header);
        dashAddLine(header);
    }

    public void addSeparator() {
        if (enableDSTelemetry) dsTelemetry.addLine(SEPARATOR_HTML);
        dashAddLine(SEPARATOR_HTML);
    }

    public void addModuleHeader(String moduleName, String stateString) {
        if (enableDSTelemetry) {
            dsTelemetry.addData(
                    htmlColor(COLOR_MODULE, htmlBold(htmlEscape(moduleName))),
                    htmlColor(COLOR_STATE, htmlEscape(stateString)));
        }
        emit(moduleName, stateString, false, true);
    }

    public DualTelemetry addDSData(String caption, Object value) {
        emit(caption, value, true, false);
        return this;
    }

    public DualTelemetry addDSData(String caption, String format, Object... args) {
        emit(caption, format, args, true, false);
        return this;
    }

    // DS only: the dashboard's HTML sanitizer has no <pre>, so the braille field map would render
    // unwrapped as one run-together line.
    public DualTelemetry addDSLine(String value) {
        if (enableDSTelemetry) dsTelemetry.addLine(value);
        return this;
    }

    public DualTelemetry addRawHtml(String caption, String htmlValue) {
        if (enableDSTelemetry) dsTelemetry.addData(fmtCaption(caption), htmlValue);
        // A bare line, not put(): keyed data reaches the graph view, and markup is not a number.
        dashAddLine(caption + ": " + htmlValue);
        return this;
    }

    public DualTelemetry addDashboardData(String caption, Object value) {
        emit(caption, value, false, true);
        return this;
    }

    public DualTelemetry addDashboardData(String caption, String format, Object... args) {
        emit(caption, format, args, false, true);
        return this;
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return emit(caption, format, args, true, true);
    }

    @Override
    public Item addData(String caption, Object value) {
        return emit(caption, value, true, true);
    }

    // The two producer overloads keep the producer live on each backend rather than resolving it
    // through emit(), which would turn a retained item into a one-shot value.
    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        if (noSinkEnabled()) return EMPTY_ITEM;
        Func<String> htmlProducer = () -> fmtValue(valueProducer.value());
        Item dsItem = enableDSTelemetry
                ? dsTelemetry.addData(fmtCaption(caption), htmlProducer)
                : null;
        Item dashItem = null;
        if (enableDashboardTelemetry) {
            if (packet != null) packet.put(caption, valueProducer.value());
            else dashItem = dashTelemetry.addData(caption, valueProducer);
        }
        return new EnhancedItem(dsItem, dashItem);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        if (noSinkEnabled()) return EMPTY_ITEM;
        Func<String> htmlProducer = () -> fmtValue(String.format(format, valueProducer.value()));
        Item dsItem = enableDSTelemetry
                ? dsTelemetry.addData(fmtCaption(caption), htmlProducer)
                : null;
        Item dashItem = null;
        if (enableDashboardTelemetry) {
            if (packet != null) packet.put(caption, String.format(format, valueProducer.value()));
            else dashItem = dashTelemetry.addData(caption, format, valueProducer);
        }
        return new EnhancedItem(dsItem, dashItem);
    }

    @Override
    public boolean removeItem(Item item) {
        Item dsItem = item;
        Item dashItem = item;
        if (item instanceof EnhancedItem) {
            EnhancedItem wrapper = (EnhancedItem) item;
            dsItem = wrapper.ds;
            dashItem = wrapper.dash;
        }
        boolean ds = enableDSTelemetry && dsItem != null && dsTelemetry.removeItem(dsItem);
        boolean dash = enableDashboardTelemetry && dashItem != null && dashTelemetry.removeItem(dashItem);
        return ds || dash;
    }

    @Override
    public void clear() {
        if (enableDSTelemetry) dsTelemetry.clear();
        if (!enableDashboardTelemetry) return;
        if (packet == null) {
            dashTelemetry.clear();
            return;
        }
        packet.getItems().clear();
        packet.getData().clear();
        // An item-less packet only blanks the view if it says it is a telemetry frame.
        packet.markTelemetryFrame();
    }

    @Override
    public void clearAll() {
        if (enableDSTelemetry) dsTelemetry.clearAll();
        if (!enableDashboardTelemetry) return;
        packetLog.clear();
        if (packet == null) {
            dashTelemetry.clearAll();
            return;
        }
        packet.getItems().clear();
        packet.getData().clear();
        packet.clearLog();
        packet.markTelemetryFrame();
    }

    // The adapter clears only the entry list it owns, which the packet path never fills, so browsers
    // keep showing the last frame unless they are cleared directly.

    @Override
    public Object addAction(Runnable action) {
        if (enableDSTelemetry) dsTelemetry.addAction(action);
        if (enableDashboardTelemetry) dashTelemetry.addAction(action);
        return action;
    }

    @Override
    public boolean removeAction(Object token) {
        boolean ds = enableDSTelemetry && dsTelemetry.removeAction(token);
        boolean dash = enableDashboardTelemetry && dashTelemetry.removeAction(token);
        return ds | dash;
    }

    @Override
    public void speak(String text) {
        if (enableDSTelemetry) dsTelemetry.speak(text);
        if (enableDashboardTelemetry) dashTelemetry.speak(text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        if (enableDSTelemetry) dsTelemetry.speak(text, languageCode, countryCode);
        if (enableDashboardTelemetry) dashTelemetry.speak(text, languageCode, countryCode);
    }

    @Override
    public boolean update() {
        ensureDisplayFormats();
        // Telemetry contract: true if transmitted. Fan-out → "any backend transmitted".
        if (!enableDSTelemetry && !enableDashboardTelemetry) return true;
        boolean ds = enableDSTelemetry && dsTelemetry.update();
        // packet != null: EnhancedOpMode sends the shared packet, so the adapter must NOT send a
        // second overlay-less one — that alternation is the field-view flicker.
        if (enableDashboardTelemetry && packet != null) replayLog(packet);
        boolean dash = enableDashboardTelemetry && (packet != null || dashTelemetry.update());
        return ds || dash;
    }

    // Like the adapter's LogAdapter.saveTo: the client shows the log of the newest packet carrying one, so every packet must.
    private void replayLog(TelemetryPacket p) {
        p.clearLog();
        Iterator<String> it = packetLogOrder == Log.DisplayOrder.OLDEST_FIRST ? packetLog.iterator() : packetLog.descendingIterator();
        while (it.hasNext()) p.addLogEntry(it.next());
    }

    @Override
    public Line addLine() {
        Line dsLine = enableDSTelemetry ? dsTelemetry.addLine() : null;
        Line dashLine = enableDashboardTelemetry ? dashTelemetry.addLine() : null;
        return new EnhancedLine(dsLine, dashLine);
    }

    @Override
    public Line addLine(String lineCaption) {
        Line dsLine = enableDSTelemetry ? dsTelemetry.addLine(lineCaption) : null;
        Line dashLine = enableDashboardTelemetry ? dashTelemetry.addLine(lineCaption) : null;
        return new EnhancedLine(dsLine, dashLine);
    }

    @Override
    public boolean removeLine(Line line) {
        Line dsLine = line;
        Line dashLine = line;
        if (line instanceof EnhancedLine) {
            EnhancedLine wrapper = (EnhancedLine) line;
            dsLine = wrapper.ds;
            dashLine = wrapper.dash;
        }
        boolean ds = enableDSTelemetry && dsLine != null && dsTelemetry.removeLine(dsLine);
        boolean dash = enableDashboardTelemetry && dashLine != null && dashTelemetry.removeLine(dashLine);
        return ds || dash;
    }

    @Override
    public boolean isAutoClear() {
        if (enableDSTelemetry) return dsTelemetry.isAutoClear();
        if (enableDashboardTelemetry) return dashTelemetry.isAutoClear();
        return true;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        if (enableDSTelemetry) dsTelemetry.setAutoClear(autoClear);
        if (enableDashboardTelemetry) dashTelemetry.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        if (enableDSTelemetry) return dsTelemetry.getMsTransmissionInterval();
        if (enableDashboardTelemetry) return dashTelemetry.getMsTransmissionInterval();
        return 0;
    }

    @Override
    public void setMsTransmissionInterval(int interval) {
        if (enableDSTelemetry) dsTelemetry.setMsTransmissionInterval(interval);
        if (enableDashboardTelemetry) dashTelemetry.setMsTransmissionInterval(interval);
    }

    @Override
    public String getItemSeparator() {
        if (enableDSTelemetry) return dsTelemetry.getItemSeparator();
        if (enableDashboardTelemetry) return dashTelemetry.getItemSeparator();
        return "";
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        if (enableDSTelemetry) dsTelemetry.setItemSeparator(itemSeparator);
        if (enableDashboardTelemetry) dashTelemetry.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        if (enableDSTelemetry) return dsTelemetry.getCaptionValueSeparator();
        if (enableDashboardTelemetry) return dashTelemetry.getCaptionValueSeparator();
        return "";
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        if (enableDSTelemetry) dsTelemetry.setCaptionValueSeparator(captionValueSeparator);
        if (enableDashboardTelemetry) dashTelemetry.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        if (enableDSTelemetry) dsTelemetry.setDisplayFormat(displayFormat);
        if (enableDashboardTelemetry) dashTelemetry.setDisplayFormat(displayFormat);
    }

    @Override
    public Log log() {
        Log dsLog = enableDSTelemetry ? dsTelemetry.log() : null;
        Log dashLog = enableDashboardTelemetry ? dashTelemetry.log() : null;
        return new CombinedLog(dsLog, dashLog);
    }

    private static class EnhancedItem implements Item {
        @Nullable private final Item ds;
        @Nullable private final Item dash;

        EnhancedItem(@Nullable Item ds, @Nullable Item dash) {
            this.ds = ds;
            this.dash = dash;
        }

        @Override public String getCaption() {
            if (ds != null) return ds.getCaption();
            if (dash != null) return dash.getCaption();
            return "";
        }

        @Override
        public Item setCaption(String caption) {
            if (ds != null) ds.setCaption(caption);
            if (dash != null) dash.setCaption(caption);
            return this;
        }

        @Override
        public Item setValue(String format, Object... args) {
            if (ds != null) ds.setValue(format, args);
            if (dash != null) dash.setValue(format, args);
            return this;
        }

        @Override
        public Item setValue(Object value) {
            if (ds != null) ds.setValue(value);
            if (dash != null) dash.setValue(value);
            return this;
        }

        @Override
        public <T> Item setValue(Func<T> valueProducer) {
            if (ds != null) ds.setValue(valueProducer);
            if (dash != null) dash.setValue(valueProducer);
            return this;
        }

        @Override
        public <T> Item setValue(String format, Func<T> valueProducer) {
            if (ds != null) ds.setValue(format, valueProducer);
            if (dash != null) dash.setValue(format, valueProducer);
            return this;
        }

        @Override
        public Item setRetained(@Nullable Boolean retained) {
            if (ds != null) ds.setRetained(retained);
            if (dash != null) dash.setRetained(retained);
            return this;
        }

        @Override public boolean isRetained() {
            if (ds != null) return ds.isRetained();
            if (dash != null) return dash.isRetained();
            return false;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            if (ds != null) ds.addData(caption, format, args);
            if (dash != null) dash.addData(caption, format, args);
            return this;
        }

        @Override
        public Item addData(String caption, Object value) {
            if (ds != null) ds.addData(caption, value);
            if (dash != null) dash.addData(caption, value);
            return this;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            if (ds != null) ds.addData(caption, valueProducer);
            if (dash != null) dash.addData(caption, valueProducer);
            return this;
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            if (ds != null) ds.addData(caption, format, valueProducer);
            if (dash != null) dash.addData(caption, format, valueProducer);
            return this;
        }
    }

    private static class EnhancedLine implements Line {
        @Nullable private final Line ds;
        @Nullable private final Line dash;

        EnhancedLine(@Nullable Line ds, @Nullable Line dash) {
            this.ds = ds;
            this.dash = dash;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            Item dsItem = ds != null ? ds.addData(caption, format, args) : null;
            Item dashItem = dash != null ? dash.addData(caption, format, args) : null;
            return new EnhancedItem(dsItem, dashItem);
        }

        @Override
        public Item addData(String caption, Object value) {
            Item dsItem = ds != null ? ds.addData(caption, value) : null;
            Item dashItem = dash != null ? dash.addData(caption, value) : null;
            return new EnhancedItem(dsItem, dashItem);
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            Item dsItem = ds != null ? ds.addData(caption, valueProducer) : null;
            Item dashItem = dash != null ? dash.addData(caption, valueProducer) : null;
            return new EnhancedItem(dsItem, dashItem);
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            Item dsItem = ds != null ? ds.addData(caption, format, valueProducer) : null;
            Item dashItem = dash != null ? dash.addData(caption, format, valueProducer) : null;
            return new EnhancedItem(dsItem, dashItem);
        }
    }

    private class CombinedLog implements Log {
        @Nullable private final Log dsLog;
        @Nullable private final Log dashLog;

        CombinedLog(Log dsLog, Log dashLog) {
            this.dsLog = dsLog;
            this.dashLog = dashLog;
        }

        @Override
        public int getCapacity() {
            if (dsLog != null) return dsLog.getCapacity();
            if (dashLog != null) return dashLog.getCapacity();
            return packetLogCapacity;
        }

        @Override
        public void setCapacity(int capacity) {
            if (dsLog != null) dsLog.setCapacity(capacity);
            if (dashLog != null) dashLog.setCapacity(capacity);
            packetLogCapacity = capacity;
            while (packetLog.size() > packetLogCapacity) packetLog.removeFirst();
        }

        @Override
        public DisplayOrder getDisplayOrder() {
            if (dsLog != null) return dsLog.getDisplayOrder();
            if (dashLog != null) return dashLog.getDisplayOrder();
            return packetLogOrder;
        }

        @Override
        public void setDisplayOrder(DisplayOrder displayOrder) {
            if (dsLog != null) dsLog.setDisplayOrder(displayOrder);
            if (dashLog != null) dashLog.setDisplayOrder(displayOrder);
            packetLogOrder = displayOrder;
        }

        @Override
        public void add(String message) {
            if (dsLog != null) dsLog.add(message);
            // On the packet path the adapter's own log never ships (its update() is suppressed), so we keep the ring.
            if (enableDashboardTelemetry && packet != null) {
                packetLog.addLast(message);
                while (packetLog.size() > packetLogCapacity) packetLog.removeFirst();
            } else if (dashLog != null) dashLog.add(message);
        }

        @Override
        public void add(String format, Object... args) {
            add(String.format(format, args));
        }

        @Override
        public void clear() {
            if (dsLog != null) dsLog.clear();
            if (dashLog != null) dashLog.clear();
            packetLog.clear();
        }
    }
}

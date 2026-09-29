package org.firstinspires.ftc.teamcode.architecture.telemetry;

import androidx.annotation.Nullable;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static org.firstinspires.ftc.teamcode.architecture.telemetry.HtmlFormatter.*;
import static org.firstinspires.ftc.teamcode.architecture.OptimizationToggles.dsTransmissionIntervalMs;
import static org.firstinspires.ftc.teamcode.architecture.OptimizationToggles.telemetryLazyFormat;

/**
 * Fan-out telemetry: structure (headers, separators, raw-HTML rows) goes to both screens as the same
 * HTML, data rows stay plain on the dashboard so its graph view keeps numbers, and the braille field
 * map is Driver Station only.
 *
 * <p>Driver Station writes are accepted only in a DS frame: a loop in which {@link #beginLoop()} found the
 * SDK's transmission interval elapsed. On other loops they are dropped, since the SDK would not send them.
 */
public class DualTelemetry implements Telemetry {
    private final Telemetry dsTelemetry;
    private final Telemetry dashTelemetry;
    private TelemetryPacket packet;
    private boolean enableDSTelemetry = true;
    private boolean enableDashboardTelemetry = true;
    private boolean dsFormatApplied = false;
    private boolean dashFormatApplied = false;
    // Open until the first update() so writes made during init() reach the SDK's post-init update().
    private boolean dsFrameOpen = true;
    private boolean dsEverSent = false;
    private long lastDSSendNs;
    private int appliedDsIntervalMs = -1;
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

    // HTML mode is a one-shot setDisplayFormat(), so re-apply it on every off→on toggle or raw tags show.
    // The dashboard adapter resets to CLASSIC at onOpModePreInit, before the Robot constructor builds this.
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

    public void setEnabled(boolean ds, boolean dash) {
        enableDSTelemetry = ds;
        enableDashboardTelemetry = dash;
    }

    /**
     * Applies OptimizationToggles.dsTransmissionIntervalMs to the Driver Station when it changed since the last call,
     * so a direct {@link #setMsTransmissionInterval} holds until the toggle is edited.
     */
    public void syncDsTransmissionInterval() {
        int interval = dsTransmissionIntervalMs;
        if (interval == appliedDsIntervalMs) return;
        if (interval < 0) {
            throw new IllegalArgumentException("OptimizationToggles.dsTransmissionIntervalMs must be >= 0, was " + interval);
        }
        dsTelemetry.setMsTransmissionInterval(interval);
        appliedDsIntervalMs = interval;
    }

    /**
     * Opens a DS frame when the SDK will transmit: TelemetryImpl sends an update() only once more than
     * getMsTransmissionInterval() ms have passed since its last send. Timed from our confirmed send, which follows
     * the SDK's timer reset, so ours never elapses first.
     */
    public void beginLoop() {
        dsFrameOpen = enableDSTelemetry
                && (!dsEverSent || (System.nanoTime() - lastDSSendNs) / 1e6 > dsTelemetry.getMsTransmissionInterval());
    }

    /** True in a loop whose DS lines will be transmitted; gate DS-only work on it. */
    public boolean isDSFrame() {
        return dsFrameOpen && enableDSTelemetry;
    }

    /** Dashboard data goes into this packet so one frame carries data and overlay; null restores the adapter. */
    public void setPacket(TelemetryPacket packet) { this.packet = packet; }

    private boolean noSinkEnabled() {
        return telemetryLazyFormat && !isDSFrame() && !enableDashboardTelemetry;
    }

    private Item emit(String caption, Object value, boolean ds, boolean dash) {
        if (noSinkEnabled()) return EMPTY_ITEM;
        Item dsItem = ds && isDSFrame()
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
        if (isDSFrame()) dsTelemetry.addLine(header);
        dashAddLine(header);
    }

    public void addSeparator() {
        if (isDSFrame()) dsTelemetry.addLine(SEPARATOR_HTML);
        dashAddLine(SEPARATOR_HTML);
    }

    public void addModuleHeader(String moduleName, String stateString) {
        if (isDSFrame()) {
            dsTelemetry.addData(
                    htmlColor(COLOR_MODULE, htmlBold(htmlEscape(moduleName))),
                    htmlColor(COLOR_STATE, htmlEscape(stateString)));
        }
        emit(moduleName, stateString, false, true);
    }

    /** Driver Station: one big red line. Dashboard: plain keyed row {@code FAULT <key>}. */
    public void addFault(String key, String message) {
        if (isDSFrame()) {
            dsTelemetry.addLine(htmlBold(htmlColor(COLOR_FAULT,
                    htmlSize(FONT_XLARGE, "FAULT " + htmlEscape(key))
                            + htmlSize(FONT_LARGE, ": " + htmlEscape(message)))));
        }
        emit("FAULT " + key, message, false, true);
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
        if (isDSFrame()) dsTelemetry.addLine(value);
        return this;
    }

    public DualTelemetry addRawHtml(String caption, String htmlValue) {
        if (isDSFrame()) dsTelemetry.addData(fmtCaption(caption), htmlValue);
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
        Item dsItem = isDSFrame()
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
        Item dsItem = isDSFrame()
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
        boolean dash = enableDashboardTelemetry && dashItem != null
                && (dashItem instanceof PacketLineItem ? ((PacketLineItem) dashItem).remove() : dashTelemetry.removeItem(dashItem));
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
        // The adapter's clear() never touches the packet, so browsers keep the last frame unless it is cleared here.
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

    /** Closes the DS frame, so the SDK's own post-loop update() is a DS no-op and cannot re-send a stale frame. */
    @Override
    public boolean update() {
        ensureDisplayFormats();
        boolean ds = false;
        if (isDSFrame()) {
            ds = dsTelemetry.update();
            // A false return means the SDK held the lines; the frame stays due and is rebuilt fresh next loop.
            if (ds) {
                dsEverSent = true;
                lastDSSendNs = System.nanoTime();
            }
        }
        dsFrameOpen = false;
        if (!enableDashboardTelemetry) return ds;
        // EnhancedOpMode sends the shared packet.
        if (packet != null) {
            replayLog(packet);
            return true;
        }
        return dashTelemetry.update() || ds;
    }

    // Like the adapter's LogAdapter.saveTo: the client shows the log of the newest packet carrying one, so every packet must.
    private void replayLog(TelemetryPacket p) {
        p.clearLog();
        Iterator<String> it = packetLogOrder == Log.DisplayOrder.OLDEST_FIRST ? packetLog.iterator() : packetLog.descendingIterator();
        while (it.hasNext()) p.addLogEntry(it.next());
    }

    @Override
    public Line addLine() {
        Line dsLine = isDSFrame() ? dsTelemetry.addLine() : null;
        return new EnhancedLine(dsLine, dashLine(null));
    }

    @Override
    public Line addLine(String lineCaption) {
        Line dsLine = isDSFrame() ? dsTelemetry.addLine(lineCaption) : null;
        return new EnhancedLine(dsLine, dashLine(lineCaption));
    }

    // On the packet path the adapter is never sent, so the line has to be written into the packet itself.
    @Nullable
    private Line dashLine(@Nullable String lineCaption) {
        if (!enableDashboardTelemetry) return null;
        if (packet != null) return new PacketLine(packet, lineCaption == null ? "" : lineCaption);
        return lineCaption == null ? dashTelemetry.addLine() : dashTelemetry.addLine(lineCaption);
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
        boolean dash = enableDashboardTelemetry && dashLine != null
                && (dashLine instanceof PacketLine ? ((PacketLine) dashLine).remove() : dashTelemetry.removeLine(dashLine));
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

    /** The Driver Station's; the dashboard's is OptimizationToggles.dashboardTransmissionIntervalMs. */
    @Override
    public int getMsTransmissionInterval() {
        return dsTelemetry.getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int interval) {
        dsTelemetry.setMsTransmissionInterval(interval);
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

    /** One packet row, rendered like the adapter's lines: the caption, then each item as caption: value. */
    private final class PacketLine implements Line {
        private final TelemetryPacket owner;
        private final TelemetryPacket.Item row;
        private final String caption;
        private final List<PacketLineItem> items = new ArrayList<>();

        PacketLine(TelemetryPacket owner, String caption) {
            this.owner = owner;
            this.caption = caption;
            row = owner.addItem(caption);
        }

        Item add(String itemCaption, Object value) {
            PacketLineItem item = new PacketLineItem(this, itemCaption, value);
            items.add(item);
            render();
            return item;
        }

        // put() would add a row of its own; putData() only feeds the graph and CSV views.
        void render() {
            StringBuilder text = new StringBuilder(caption);
            for (int i = 0; i < items.size(); i++) {
                PacketLineItem item = items.get(i);
                if (i > 0) text.append(dashTelemetry.getItemSeparator());
                text.append(item.caption).append(dashTelemetry.getCaptionValueSeparator()).append(item.value);
                owner.putData(item.caption, item.value);
            }
            row.setValue(text.toString());
        }

        boolean remove() {
            return owner.getItems().remove(row);
        }

        boolean remove(PacketLineItem item) {
            if (!items.remove(item)) return false;
            render();
            return true;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return add(caption, String.format(format, args));
        }

        @Override
        public Item addData(String caption, Object value) {
            return add(caption, value);
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            return add(caption, valueProducer.value());
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return add(caption, String.format(format, valueProducer.value()));
        }
    }

    /** Producers are resolved once: the packet is rebuilt every loop, so there is nothing to retain. */
    private static final class PacketLineItem implements Item {
        private final PacketLine line;
        private String caption;
        private String value;

        PacketLineItem(PacketLine line, String caption, Object value) {
            this.line = line;
            this.caption = caption;
            this.value = String.valueOf(value);
        }

        boolean remove() {
            return line.remove(this);
        }

        private Item set(Object newValue) {
            value = String.valueOf(newValue);
            line.render();
            return this;
        }

        @Override public String getCaption() { return caption; }

        @Override
        public Item setCaption(String caption) {
            this.caption = caption;
            line.render();
            return this;
        }

        @Override public Item setValue(String format, Object... args) { return set(String.format(format, args)); }

        @Override public Item setValue(Object value) { return set(value); }

        @Override public <T> Item setValue(Func<T> valueProducer) { return set(valueProducer.value()); }

        @Override public <T> Item setValue(String format, Func<T> valueProducer) {
            return set(String.format(format, valueProducer.value()));
        }

        @Override public Item setRetained(@Nullable Boolean retained) { return this; }

        @Override public boolean isRetained() { return false; }

        @Override public Item addData(String caption, String format, Object... args) { return line.addData(caption, format, args); }

        @Override public Item addData(String caption, Object value) { return line.addData(caption, value); }

        @Override public <T> Item addData(String caption, Func<T> valueProducer) { return line.addData(caption, valueProducer); }

        @Override public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return line.addData(caption, format, valueProducer);
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

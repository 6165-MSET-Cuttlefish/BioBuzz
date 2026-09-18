package org.firstinspires.ftc.teamcode.architecture.telemetry;

/** Renders a 2D pixel grid as Unicode Braille (U+2800..U+28FF, 2×4 pixels per glyph) in HTML for the DS field map. */
public class FieldMapRenderer {

    private int width;
    private int height;
    private boolean[][] pixels;
    private String[][] cellColors;
    private boolean[][] snapshotPixels;
    private String[][] snapshotColors;
    private double scaleX;
    private double scaleY;

    public FieldMapRenderer(int width, int height) {
        setSize(width, height);
    }

    public final void setSize(int width, int height) {
        this.width = width;
        this.height = height;
        this.scaleX = width / 144.0;
        this.scaleY = height / 144.0;
        this.pixels = new boolean[height][width];
        this.cellColors = new String[(height + 3) / 4][(width + 1) / 2];
        // Snapshot was sized to the old dimensions; void it so restore() can't mismatch bounds.
        this.snapshotPixels = null;
        this.snapshotColors = null;
    }

    public void drawFieldLayout() {
        clear();

        int w = width - 1;
        int h = height - 1;
        String grid = "#666666";

        drawRect(0, 0, w, h);

        int tiles = 6;
        for (int i = 1; i < tiles; i++) {
            int x = alignXToGlyph((int) ((i / (double) tiles) * w));
            int y = (int) ((i / (double) tiles) * h);
            drawLine(x, 0, x, h, grid);
            drawLine(0, y, w, y, grid);
        }
    }

    public void drawRobot(double centerXInches, double centerYInches, double headingRadians, String color) {
        int px = toPxX(centerXInches);
        int py = toPxY(centerYInches);
        int r  = toPxX(18 / 2.0);

        drawCircle(px, py, r, color);

        int endX = (int) Math.round(px + Math.cos(headingRadians) * r);
        int endY = (int) Math.round(py + Math.sin(headingRadians) * r);
        drawLine(px, py, endX, endY, color);
    }

    public void drawPoint(double xInches, double yInches, String color) {
        int px = toPxX(xInches);
        int py = toPxY(yInches);
        int r = Math.max(1, toPxX(5 / 2.0));
        drawCircle(px, py, r, color);
    }

    public void clear() {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) pixels[y][x] = false;
        }
        for (int y = 0; y < cellColors.length; y++) {
            for (int x = 0; x < cellColors[0].length; x++) cellColors[y][x] = null;
        }
    }

    public void snapshot() {
        snapshotPixels = new boolean[height][width];
        snapshotColors = new String[cellColors.length][cellColors[0].length];
        for (int y = 0; y < height; y++)
            System.arraycopy(pixels[y], 0, snapshotPixels[y], 0, width);
        for (int y = 0; y < cellColors.length; y++)
            System.arraycopy(cellColors[y], 0, snapshotColors[y], 0, cellColors[0].length);
    }

    public void restore() {
        if (snapshotPixels == null) return;
        for (int y = 0; y < height; y++)
            System.arraycopy(snapshotPixels[y], 0, pixels[y], 0, width);
        for (int y = 0; y < cellColors.length; y++)
            System.arraycopy(snapshotColors[y], 0, cellColors[y], 0, cellColors[0].length);
    }

    public void setPixel(int x, int y, String color) {
        int py = (height - 1) - y;
        if (x < 0 || py < 0 || x >= width || py >= height) return;
        pixels[py][x] = true;
        cellColors[py / 4][x / 2] = color;
    }

    public void setPixel(int x, int y) {
        int py = (height - 1) - y;
        if (x < 0 || py < 0 || x >= width || py >= height) return;
        pixels[py][x] = true;
    }

    public void drawHorizontal(int x1, int x2, int y) {
        for (int x = x1; x <= x2; x++) setPixel(x, y);
    }

    public void drawHorizontal(int x1, int x2, int y, String color) {
        for (int x = x1; x <= x2; x++) setPixel(x, y, color);
    }

    public void drawVertical(int y1, int y2, int x) {
        for (int y = y1; y <= y2; y++) setPixel(x, y);
    }

    public void drawVertical(int y1, int y2, int x, String color) {
        for (int y = y1; y <= y2; y++) setPixel(x, y, color);
    }

    public void drawRect(int x, int y, int w, int h) {
        drawHorizontal(x, x + w, y);
        drawHorizontal(x, x + w, y + h);
        drawVertical(y, y + h, x);
        drawVertical(y, y + h, x + w);
    }

    public void drawRect(int x, int y, int w, int h, String color) {
        drawHorizontal(x, x + w, y, color);
        drawHorizontal(x, x + w, y + h, color);
        drawVertical(y, y + h, x, color);
        drawVertical(y, y + h, x + w, color);
    }

    public void drawLine(int x0, int y0, int x1, int y1) {
        drawLineImpl(x0, y0, x1, y1, null);
    }

    public void drawLine(int x0, int y0, int x1, int y1, String color) {
        drawLineImpl(x0, y0, x1, y1, color);
    }

    private void drawLineImpl(int x0, int y0, int x1, int y1, String color) {
        int dx = Math.abs(x1 - x0);
        int dy = Math.abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (color == null) setPixel(x0, y0);
            else setPixel(x0, y0, color);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx)  { err += dx; y0 += sy; }
        }
    }

    private void drawCircle(int cx, int cy, int r, String color) {
        int x = r;
        int y = 0;
        int err = 0;

        while (x >= y) {
            setPixel(cx + x, cy + y, color);
            setPixel(cx + y, cy + x, color);
            setPixel(cx - y, cy + x, color);
            setPixel(cx - x, cy + y, color);
            setPixel(cx - x, cy - y, color);
            setPixel(cx - y, cy - x, color);
            setPixel(cx + y, cy - x, color);
            setPixel(cx + x, cy - y, color);

            if (err <= 0) { y++; err += 2 * y + 1; }
            if (err > 0)  { x--; err -= 2 * x + 1; }
        }
    }

    public String renderHtml() {
        int cellRows = (height + 3) / 4;
        int cellCols = (width + 1) / 2;
        StringBuilder out = new StringBuilder(cellRows * cellCols * 4);
        out.append("<pre style='line-height:1; letter-spacing:0;'>");
        for (int y = 0; y < height; y += 4) {
            int cy = y / 4;
            String runColor = null;
            StringBuilder run = new StringBuilder();
            for (int x = 0; x < width; x += 2) {
                char b = braille(x, y);
                String color = cellColors[cy][x / 2];
                if (!strEq(color, runColor)) {
                    flushRun(out, run, runColor);
                    runColor = color;
                }
                run.append(b);
            }
            flushRun(out, run, runColor);
            out.append('\n');
        }
        out.append("</pre>");
        return out.toString();
    }

    private int toPxX(double inches) { return (int) Math.round(inches * scaleX); }
    private int toPxY(double inches) { return (int) Math.round(inches * scaleY); }

    private boolean isTrue(int x, int y) {
        if (x < 0 || y < 0 || x >= width || y >= height) return false;
        return pixels[y][x];
    }

    private char braille(int x, int y) {
        int code = 0;
        if (isTrue(x, y))         code |= 1;
        if (isTrue(x, y + 1))     code |= 2;
        if (isTrue(x, y + 2))     code |= 4;
        if (isTrue(x + 1, y))     code |= 8;
        if (isTrue(x + 1, y + 1)) code |= 16;
        if (isTrue(x + 1, y + 2)) code |= 32;
        if (isTrue(x, y + 3))     code |= 64;
        if (isTrue(x + 1, y + 3)) code |= 128;
        return (char) (0x2800 + code);
    }

    /** Snap x to an even pixel — Braille glyphs are 2 pixels wide. */
    private static int alignXToGlyph(int x) {
        return (x / 2) * 2;
    }

    private static void flushRun(StringBuilder out, StringBuilder run, String color) {
        if (run.length() == 0) return;
        if (color != null) {
            out.append("<font color='").append(color).append("'>")
               .append(run).append("</font>");
        } else {
            out.append(run);
        }
        run.setLength(0);
    }

    private static boolean strEq(String a, String b) {
        if (a == b) return true;
        if (a == null || b == null) return false;
        return a.equals(b);
    }
}

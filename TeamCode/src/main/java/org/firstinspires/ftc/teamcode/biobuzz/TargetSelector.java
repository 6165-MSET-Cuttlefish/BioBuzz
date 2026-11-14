package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField.Cell;

public final class TargetSelector {

    private Cell target = Cell.STARTS_UP;
    private boolean lastTipped;

    public Cell targetCell() {
        return target;
    }

    public void set(Cell cell) {
        target = cell;
    }

    public void toggle() {
        target = target.other();
    }

    /** Flips on each rising edge of a TIP verdict (e.g. {@code CellTipCamera.isTipped()}). */
    public void onTipVerdict(boolean tipped) {
        if (tipped && !lastTipped) toggle();
        lastTipped = tipped;
    }

    // Within bandIn of the midline it keeps its choice, or pose noise there swings the turrets between the CELLs.
    public void followHalf(double robotYIn, double fieldWidthIn, boolean blue, double bandIn) {
        double fromMidline = robotYIn - fieldWidthIn / 2;
        if (Math.abs(fromMidline) > bandIn) target = BioBuzzField.cellFacing(fromMidline < 0, blue);
    }

    public Cell[] forShooters(int count) {
        Cell[] cells = new Cell[count];
        for (int i = 0; i < count; i++) cells[i] = target;
        return cells;
    }
}

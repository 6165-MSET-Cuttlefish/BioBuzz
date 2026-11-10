package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.BioBuzzField.Cell;

/** Which of this alliance's two CELLS both turrets aim at. */
public final class TargetSelector {

    private Cell up = Cell.STARTS_UP;
    private boolean lastTipped;

    public Cell upCell() {
        return up;
    }

    public void set(Cell cell) {
        up = cell;
    }

    /** The driver's override. */
    public void toggle() {
        up = up.other();
    }

    /** Flips on each rising edge of a TIP verdict (e.g. {@code CellTipCamera.isTipped()}). */
    public void onTipVerdict(boolean tipped) {
        if (tipped && !lastTipped) toggle();
        lastTipped = tipped;
    }

    /**
     * Picks the CELL that faces the half of the field the robot is in, the only one it can shoot into from there.
     * Within {@code bandIn} of the midline it keeps its choice, or pose noise there swings the turrets between the two.
     */
    public void followHalf(double robotYIn, double fieldWidthIn, boolean blue, double bandIn) {
        double fromMidline = robotYIn - fieldWidthIn / 2;
        if (Math.abs(fromMidline) > bandIn) up = BioBuzzField.cellFacing(fromMidline < 0, blue);
    }

    public Cell[] forShooters(int count) {
        Cell[] cells = new Cell[count];
        for (int i = 0; i < count; i++) cells[i] = up;
        return cells;
    }
}

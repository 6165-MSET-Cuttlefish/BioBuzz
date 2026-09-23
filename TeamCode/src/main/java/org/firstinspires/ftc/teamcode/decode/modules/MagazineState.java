package org.firstinspires.ftc.teamcode.decode.modules;

public class MagazineState {
    public enum ArtifactColor {
        PURPLE('P'),
        GREEN('G'),
        EMPTY('E');

        private final char symbol;

        ArtifactColor(char symbol) {
            this.symbol = symbol;
        }

        public char getSymbol() {
            return symbol;
        }
    }

    private final ArtifactColor position1;
    private final ArtifactColor position2;
    private final ArtifactColor position3;

    public MagazineState(
            ArtifactColor position1, ArtifactColor position2, ArtifactColor position3) {
        this.position1 = position1;
        this.position2 = position2;
        this.position3 = position3;
    }

    public ArtifactColor getPosition1() {
        return position1;
    }

    public ArtifactColor getPosition2() {
        return position2;
    }

    public ArtifactColor getPosition3() {
        return position3;
    }

    public ArtifactColor getPosition(int position) {
        switch (position) {
            case 1:
                return position1;
            case 2:
                return position2;
            case 3:
                return position3;
            default:
                return ArtifactColor.EMPTY;
        }
    }

    public String toPattern() {
        return "" + position1.getSymbol() + position2.getSymbol() + position3.getSymbol();
    }

    public static MagazineState fromPattern(String pattern) {
        if (pattern == null || pattern.length() != 3) {
            throw new IllegalArgumentException(
                    "MagazineState pattern must be exactly 3 chars, got: " + pattern);
        }
        return new MagazineState(charToArtifactColor(pattern.charAt(0)),
                charToArtifactColor(pattern.charAt(1)), charToArtifactColor(pattern.charAt(2)));
    }

    private static ArtifactColor charToArtifactColor(char c) {
        switch (c) {
            case 'G':
                return ArtifactColor.GREEN;
            case 'P':
                return ArtifactColor.PURPLE;
            default:
                return ArtifactColor.EMPTY;
        }
    }

    public boolean isFull() {
        return position1 != ArtifactColor.EMPTY && position2 != ArtifactColor.EMPTY
                && position3 != ArtifactColor.EMPTY;
    }

    public boolean isEmpty() {
        return position1 == ArtifactColor.EMPTY && position2 == ArtifactColor.EMPTY
                && position3 == ArtifactColor.EMPTY;
    }

    public int countColor(ArtifactColor color) {
        int count = 0;
        if (position1 == color)
            count++;
        if (position2 == color)
            count++;
        if (position3 == color)
            count++;
        return count;
    }

    @Override
    public String toString() {
        return String.format("MagazineState[%s, %s, %s]", position1, position2, position3);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (!(obj instanceof MagazineState))
            return false;
        MagazineState other = (MagazineState) obj;
        return position1 == other.position1 && position2 == other.position2
                && position3 == other.position3;
    }

    @Override
    public int hashCode() {
        int result = position1.hashCode();
        result = 31 * result + position2.hashCode();
        result = 31 * result + position3.hashCode();
        return result;
    }
}

package org.firstinspires.ftc.teamcode.opmodes.auto.modular;

public final class TwoTipPlan {
    public enum Hive { CLOSE, FAR }
    public enum Mode {
        SOLO("Solo: both hives"),
        PARTNER_CLOSE("Partner: close hive only"),
        PARTNER_FAR("Partner: far hive only");
        private final String label;
        Mode(String label) { this.label = label; }
        public boolean isPartner() { return this != SOLO; }
        public Hive firstHive(Hive startSide) {
            return this == PARTNER_CLOSE ? Hive.CLOSE : this == PARTNER_FAR ? Hive.FAR : startSide;
        }
        @Override public String toString() { return label; }
    }
    public enum Pickup {
        GARDEN("Garden"),
        FAR_FLOWER("Flower (46.83, 137.89)"),
        SIDE_FLOWER("Flower (2.35, 46.08)");
        private final String label;
        Pickup(String label) { this.label = label; }
        @Override public String toString() { return label; }
    }

    private final int maxAttempts;
    private final int[] attempts = new int[2];
    private final boolean[] tipped = new boolean[2];

    public TwoTipPlan(int retries) {
        if (retries < 0 || retries > 3) throw new IllegalArgumentException("Retries must be 0..3");
        maxAttempts = retries + 1;
    }

    public void record(Hive hive, boolean likelyTipped) {
        attempts[hive.ordinal()]++;
        tipped[hive.ordinal()] |= likelyTipped;
    }

    /** Retry the current hive first; advance after a tip or the configured attempt limit. */
    public Hive next(Hive current) {
        return next(current, Mode.SOLO);
    }

    /** Partner mode can reload its selected hive, but never targets the partner's other hive. */
    public Hive next(Hive current, Mode mode) {
        if (mode.isPartner()) {
            Hive selected = mode.firstHive(current);
            return available(selected) ? selected : null;
        }
        if (available(current)) return current;
        Hive other = current == Hive.CLOSE ? Hive.FAR : Hive.CLOSE;
        return available(other) ? other : null;
    }

    private boolean available(Hive hive) {
        return !tipped[hive.ordinal()] && attempts[hive.ordinal()] < maxAttempts;
    }

    public String summary() {
        return "Close " + attempts[0] + (tipped[0] ? " tipped" : " unconfirmed")
                + " / Far " + attempts[1] + (tipped[1] ? " tipped" : " unconfirmed");
    }
}

package frc.robot.util;

public class CRT {

    private static final int G_TURRET = 200, G_DRIVE = 21, G_ENC = 19;

    public static double getTurretAngleDeg(double enc1, double enc2, double enc2Offset) {
        double edFrac = enc1; // 0.0 - 1.0
        double eeFrac = enc2 - enc2Offset;

        int rd = (int) Math.round(edFrac * G_DRIVE) % G_DRIVE;
        int re = (int) Math.round(eeFrac * G_ENC)   % G_ENC;

        // CRT: solve x ≡ rd (mod G_DRIVE), x ≡ re (mod G_ENC)
        // gcd(21,19)=1, so guaranteed unique solution mod 399
        int inv = modInverse(G_DRIVE % G_ENC, G_ENC); // precompute: inv(21 mod 19, 19) = inv(2,19) = 10
        int t   = ((re - rd) % G_ENC * inv % G_ENC + G_ENC) % G_ENC;
        int x   = rd + G_DRIVE * t;
        x = ((x % (G_DRIVE * G_ENC)) + G_DRIVE * G_ENC) % (G_DRIVE * G_ENC);

        return (x / (double) G_TURRET) * 360.0 % 360.0;
    }

    private static int modInverse(int a, int m) {
        // Extended Euclidean — works for small m
        a = ((a % m) + m) % m;
        for (int x = 1; x < m; x++)
            if ((a * x) % m == 1) return x;
        return -1; // should never hit with coprime gears
    }
}
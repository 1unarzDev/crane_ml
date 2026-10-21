using System;
using System.Collections;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Sim.Physics.Aerial;

namespace Sim.Tests.Aerial {
    [Serializable] public class TumbleOracle { public string profile; public double dt, flipTime; public double[] omega0, inertiaDiag; }

    /// <summary>
    /// Torque-free tumble about the intermediate axis (Dzhanibekov flip) of the firefly inertia, driven
    /// through the real Rigidbody and CraneAerialRotorV2Plant, against the Python oracle's zero-crossing
    /// time of omega_y. Without the explicit -w x (I w) torque PhysX leaves omega constant (no flip).
    /// Tolerance: the plant applies the gyroscopic torque explicitly once per step and PhysX integrates it
    /// semi-implicitly, so the instability growth rate carries an O(lambda*dt) first-order error
    /// (lambda ~ 1.2 /s, dt = 2.5 ms -> ~0.3 %), plus one-step sampling. The flip time depends on the
    /// growth over ln(3/0.02) ~ 5 e-folds, so we allow 3 % of the oracle time (~0.13 s), which is still
    /// far below the no-term behaviour (no flip at all within 2x the oracle time).
    /// </summary>
    [Category("AerialGyro")]
    public class AerialTumblePlayTests {
        [UnityTest] public IEnumerator IntermediateAxisFlipTimeMatchesOracle() {
            var oracle = JsonUtility.FromJson<TumbleOracle>(File.ReadAllText(
                Path.Combine(Application.dataPath, "Tests", "PlayMode", "tumble_flip_oracle.json")));
            float oldDt = Time.fixedDeltaTime; Vector3 oldG = UnityEngine.Physics.gravity;
            Time.fixedDeltaTime = (float)oracle.dt;
            UnityEngine.Physics.gravity = Vector3.zero;
            var go = new GameObject("tumble");
            var rb = go.AddComponent<Rigidbody>();
            rb.useGravity = false; rb.angularDamping = 0f; rb.linearDamping = 0f; rb.maxAngularVelocity = 100f;
            string profile = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Tools", "Aerial", "profiles", oracle.profile + ".json"));
            var plant = CraneAerialRotorV2Plant.TryInstall(go, new[] { "--crane-aerial-profile=" + profile });
            Assert.IsNotNull(plant);
            // FLU body rate (a, b, c) -> Unity body-axis raw triple (b, -c, -a); identity rotation so world == body.
            double[] w = oracle.omega0;
            rb.angularVelocity = new Vector3((float)w[1], (float)-w[2], (float)-w[0]);
            double prev = w[1], flip = -1; int n = 0;
            int maxSteps = (int)(2 * oracle.flipTime / oracle.dt);
            while (n < maxSteps && flip < 0) {
                yield return new WaitForFixedUpdate();
                n++;
                Vector3 wu = Quaternion.Inverse(rb.rotation) * rb.angularVelocity;
                double wy = wu.x;                                    // pseudovector: FLU y = +Unity right
                if (prev > 0 && wy <= 0) flip = (n - 1 + prev / (prev - wy)) * oracle.dt;
                prev = wy;
            }
            UnityEngine.Object.Destroy(go);
            Time.fixedDeltaTime = oldDt; UnityEngine.Physics.gravity = oldG;
            Debug.Log($"AERIAL_TUMBLE flip={flip:F4} oracle={oracle.flipTime:F4} relErr={(flip - oracle.flipTime) / oracle.flipTime:E3}");
            Assert.Greater(flip, 0, "no flip observed (gyroscopic torque missing?)");
            Assert.AreEqual(oracle.flipTime, flip, 0.03 * oracle.flipTime);
        }
    }
}

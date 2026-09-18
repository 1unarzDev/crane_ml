using System;
using UnityEngine;

namespace Sim.Utils {
    [Serializable]
    public class Pid {
        public float Kp;
        public float Ki;
        public float Kd;
        public float bound;
        private float integral, prev;

        public Pid(float Kp, float Ki, float Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public float Run(float error, float dt) {
            float change = (error - prev) / dt;
            prev = error;
            integral += error * dt;
            integral = Mathf.Clamp(integral, -bound, bound);
            return Kp * error + Ki * integral + Kd * change;
        }

        public Pid Clone() =>
            new(Kp, Ki, Kd);

        public void Reset() {
            integral = 0;
            prev = 0;
        }
    }
}

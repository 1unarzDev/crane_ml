namespace Sim.Course {
    /// <summary>The speed challenge's red/green light sign, read by the boat's camera. Its
    /// program may toggle (opt-in); the speed task's direction source may be this sign.</summary>
    public sealed class LightSign : StatefulCourseObject {
        public override string Kind => "light_sign";
        protected override void Render(string value) => Tint(Slot(), IndicatorLight.ColorOf(value), true);
    }
}

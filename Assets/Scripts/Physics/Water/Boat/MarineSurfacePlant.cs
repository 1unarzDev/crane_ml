using System.Collections.Generic;
using System.Linq;
using Sim.Actuators.Motors;
using Sim.Physics.Core;
using Sim.Physics.Water.Dynamics;
using Sim.Physics.Water.Statics;
using Sim.Utils;
using UnityEngine;

namespace Sim.Physics.Water.Boat {
    /// <summary>
    /// The marine surface plant: water medium, <see cref="MarineHydrodynamics"/>, buoyancy, the
    /// thrusters (by name) and planar <see cref="AddedMassCoupling"/> on an articulation tree,
    /// applied at the composite centre of mass.
    /// </summary>
    public sealed class MarineSurfacePlant : PlantBase {
        private ArticulationBody[] links, hullLinks = new ArticulationBody[0];
        public MarineSurfaceParameters Parameters { get; private set; }
        public MarinePlanarModel Model { get; private set; }

        /// <param name="rigidMass">Composite mass Unity integrates [kg].</param>
        /// <param name="rigidIzz">Composite yaw inertia about the CG Unity integrates [kg m²].</param>
        public void Configure(MarineSurfaceParameters parameters, ArticulationBody root, IBodyFrame bodyFrame,
            IMedium water, IReadOnlyList<Thruster> thrusters, Buoyancy buoyancy, IEnumerable<IForceModel> extra,
            float rigidMass, float rigidIzz) {
            Parameters = parameters;
            links = root.GetComponentsInChildren<ArticulationBody>();
            body = new ArticulationBodyAdapter(root);
            frame = bodyFrame;
            medium = water;
            Model = parameters.PlanarModel(rigidMass, rigidIzz);
            coupling = new AddedMassCoupling(Model);
            forces.Clear();
            forces.Add(new MarineHydrodynamics(Model, parameters.heave.Axis, parameters.roll.Axis, parameters.pitch.Axis,
                new Vector3(parameters.envelope.surge, parameters.envelope.sway, parameters.envelope.yaw)));
            if (buoyancy != null) {
                buoyancy.PlantDriven = true;
                forces.Add(buoyancy);
            }
            forces.AddRange(extra.Where(f => f != null));
            // Contacts act outside τ_h; scale their mass so a hit sees M_A too (DockBumpTests:
            // PhysX honours the per-contact scale on articulation links).
            hullLinks = links.Where(l => l.isRoot || l.jointType == ArticulationJointType.FixedJoint).ToArray();
            Collider[] colliders = root.GetComponentsInChildren<Collider>();
            foreach (ArticulationBody link in hullLinks)
                AddedMassContacts.Register(link, colliders.Where(c => c.attachedArticulationBody == link).ToArray(),
                    rigidMass, rigidIzz, parameters.addedMass.a11, parameters.addedMass.a22, parameters.addedMass.a66);
            actuators.Clear();
            var cgOffset = new Vector3(parameters.comBow, parameters.comPort, 0f);
            foreach (ThrusterSpec spec in parameters.thrusters) {
                Thruster thruster = thrusters.First(t => Matches(t, spec.name));
                thruster.PlantDriven = true;
                actuators.Add(new ThrusterActuator(thruster, spec) { CenterOfMassOffset = cgOffset });
            }
        }

        /// <summary>A profile thruster name binds the thruster link <c>&lt;name&gt;_propeller_link</c>.</summary>
        public static bool Matches(Thruster thruster, string name) =>
            thruster.name == name || thruster.name.StartsWith(name + "_");

        protected override void FixedUpdate() {
            if (frame != null) {
                Vector3 bow = frame.UnityLinear(Vector3.right), port = frame.UnityLinear(Vector3.up);
                foreach (ArticulationBody link in hullLinks) AddedMassContacts.SetHeading(link, bow, port);
            }
            base.FixedUpdate();
        }

        private void OnDestroy() {
            foreach (ArticulationBody link in hullLinks) if (link != null) AddedMassContacts.Unregister(link);
        }

        protected override Vector3 CenterOfMass() {
            float mass = 0f;
            Vector3 sum = Vector3.zero;
            foreach (ArticulationBody link in links) {
                mass += link.mass;
                sum += link.mass * link.worldCenterOfMass;
            }
            return sum / mass;
        }
    }
}

using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;

namespace Sim.Physics.Water.Dynamics {
    /// <summary>
    /// Contact forces act outside τ_h, so on their own they see only M_RB. This scales a
    /// registered body's per-contact inverse mass by m/(m + nᵀM_A,xy n) along the horizontal
    /// normal n and its inverse inertia by I_z/(I_z + a66), for horizontal normals only, through Physics.ContactModifyEvent.
    /// The callback may run off the main thread: it reads only values set by
    /// <see cref="SetHeading"/>, never a Transform. DockBumpTests measures whether PhysX honours
    /// the scale for ArticulationBody links.
    /// </summary>
    public static class AddedMassContacts {
        private sealed class Entry {
            public float Mass, Izz, A11, A22, A66;
            public Vector3 Bow = Vector3.forward, Port = Vector3.left;
        }

        private static readonly Dictionary<EntityId, Entry> bodies = new();
        private static bool subscribed;

        public static void Register(Component body, Collider[] colliders, float mass, float izz,
            float a11, float a22, float a66) {
            foreach (Collider collider in colliders) collider.hasModifiableContacts = true;
            bodies[body.GetEntityId()] = new Entry { Mass = mass, Izz = izz, A11 = a11, A22 = a22, A66 = a66 };
            if (!subscribed) {
                UnityEngine.Physics.ContactModifyEvent += Modify;
                subscribed = true;
            }
        }

        public static void Unregister(Component body) => bodies.Remove(body.GetEntityId());

        /// <summary>Body bow and port axes in Unity world, refreshed each tick on the main thread.</summary>
        public static void SetHeading(Component body, Vector3 bow, Vector3 port) {
            if (!bodies.TryGetValue(body.GetEntityId(), out Entry e)) return;
            e.Bow = bow;
            e.Port = port;
        }

        /// <summary>A contact is horizontal when its unit normal is within 30° of the water plane
        /// (|n·up| ≤ 0.5). Only those get the added-mass scales: M_A is planar here, and a grounding
        /// or deck contact keeps the full rigid-body response.</summary>
        public const float HorizontalMaxUp = 0.5f;

        public static bool IsHorizontal(Vector3 normal) => Mathf.Abs(normal.y) <= HorizontalMaxUp;

        /// <summary>m/(m + nᵀM_A n) for a world normal n; 1 when n is not horizontal.</summary>
        public static float MassScale(float mass, float a11, float a22, Vector3 bow, Vector3 port, Vector3 normal) {
            if (!IsHorizontal(normal)) return 1f;
            float nb = Vector3.Dot(normal, bow), np = Vector3.Dot(normal, port);
            return mass / (mass + a11 * nb * nb + a22 * np * np);
        }

        /// <summary>I_z/(I_z + a66) for a horizontal normal, else 1.</summary>
        public static float InertiaScale(float izz, float a66, Vector3 normal) =>
            IsHorizontal(normal) ? izz / (izz + a66) : 1f;

        private static void Modify(PhysicsScene scene, NativeArray<ModifiableContactPair> pairs) {
            for (int i = 0; i < pairs.Length; i++) {
                ModifiableContactPair pair = pairs[i];
                if (pair.contactCount == 0) continue;
                bool self = bodies.TryGetValue(pair.bodyEntityId, out Entry a);
                bool other = bodies.TryGetValue(pair.otherBodyEntityId, out Entry b);
                if (!self && !other) continue;
                Vector3 normal = pair.GetNormal(0);
                ModifiableMassProperties props = pair.massProperties;
                if (self) {
                    props.inverseMassScale = MassScale(a.Mass, a.A11, a.A22, a.Bow, a.Port, normal);
                    props.inverseInertiaScale = InertiaScale(a.Izz, a.A66, normal);
                }
                if (other) {
                    props.otherInverseMassScale = MassScale(b.Mass, b.A11, b.A22, b.Bow, b.Port, normal);
                    props.otherInverseInertiaScale = InertiaScale(b.Izz, b.A66, normal);
                }
                pair.massProperties = props;
                pairs[i] = pair;
            }
        }
    }
}

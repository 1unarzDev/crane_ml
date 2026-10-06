using UnityEngine;

namespace Sim.Physics.Land {
    /// <summary>Render-only rotation. Contact wheels inherit physical joint rotation;
    /// the original force model uses its explicitly virtual shaft estimate.</summary>
    public sealed class CraneCampusWheelVisual : MonoBehaviour {
        public CraneCampusRobotModel Model;
        public DifferentialDriveDynamics Drive;
        public bool Left;
        void FixedUpdate() {
            if(Model==null||Model.ContactWheels||Drive==null)return;
            float shaftSpeed=Model.WheelLinearSpeed+(Left?1:-1)*Model.WheelAngularSpeed*Drive.WheelSeparation/2;
            transform.Rotate(Vector3.right,shaftSpeed/Drive.WheelRadius*Mathf.Rad2Deg*Time.fixedDeltaTime,Space.Self);
        }
    }
}

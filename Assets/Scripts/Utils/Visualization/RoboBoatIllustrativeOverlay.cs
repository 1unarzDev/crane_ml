using UnityEngine;

namespace Sim.Visualization
{
    /// <summary>Constructed, in-sim teaching overlay. Never represents ROS telemetry.</summary>
    public sealed class RoboBoatIllustrativeOverlay : MonoBehaviour
    {
        [SerializeField] private Color pathColor = new Color(0.1f, 0.95f, 1f, 1f);
        private Material Make(Color c) { var m = new Material(Shader.Find("Unlit/Color")); m.color = c; return m; }
        private void Start()
        {
            var root = new GameObject("ILLUSTRATIVE ONLY — constructed evidence overlay");
            var line = root.AddComponent<LineRenderer>(); line.material = Make(pathColor); line.widthMultiplier = .08f; line.positionCount = 9;
            var pts = new[] { new Vector3(-3f,.8f,2f), new Vector3(-2f,.8f,4f), new Vector3(-.5f,.8f,5f), new Vector3(1f,.8f,4f), new Vector3(2f,.8f,6f), new Vector3(4f,.8f,8f), new Vector3(6f,.8f,9f) }; line.SetPositions(pts);
            for (int i=0;i<12;i++) { var c=GameObject.CreatePrimitive(PrimitiveType.Cube); c.name="Illustrative costmap cell"; c.transform.position=new Vector3(-2+i%6*.8f,.35f,3+i/6*.8f); c.transform.localScale=new Vector3(.7f,.18f,.7f); c.GetComponent<Renderer>().material=Make(new Color(1f,.35f,.05f,.35f)); }
            var arrow = GameObject.CreatePrimitive(PrimitiveType.Cube); arrow.name="Illustrative cmd_vel arrow"; arrow.transform.position=new Vector3(-2,.9f,2.5f); arrow.transform.localScale=new Vector3(2f,.14f,.22f); arrow.transform.rotation=Quaternion.Euler(0,25,0); arrow.GetComponent<Renderer>().material=Make(Color.magenta);
            var contact = GameObject.CreatePrimitive(PrimitiveType.Sphere); contact.name="Illustrative contact marker"; contact.transform.position=new Vector3(1,1.1f,4); contact.transform.localScale=Vector3.one*.7f; contact.GetComponent<Renderer>().material=Make(Color.red);
            var label = new GameObject("Illustrative evidence response").AddComponent<TextMesh>(); label.text="SCHEMATIC / ILLUSTRATIVE\nE1 plan + costmap → E2 cmd_vel → E3 observed deviation\nConstructed contact; no ROS telemetry consumed.\n‘High cost and observed deviation are consistent with contact;\nforce and thruster feedback remain unknown.’"; label.fontSize=38; label.characterSize=.02f; label.color=Color.black; label.anchor=TextAnchor.UpperLeft; label.transform.position=new Vector3(-5,3,5); label.transform.rotation=Quaternion.identity;
        }
    }
}

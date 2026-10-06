using System;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace Sim.Physics.Land {
    /// <summary>Fixed-step repeatable commands also usable as the real-robot trial specification.</summary>
    public sealed class CraneCampusCalibration:MonoBehaviour {
        [Serializable] sealed class CommandEvidence { public float linear,angular; }
        [Serializable] sealed class ProgramEvidence { public string trial;public float rotationDegrees; }
        CraneIndustrialCampus campus;DifferentialDriveDynamics drive;string trial;
        float rotationDegrees=180,lastLinear=float.NaN,lastAngular=float.NaN;
        public void Configure(CraneIndustrialCampus c,DifferentialDriveDynamics d,string name){
            campus=c;drive=d;trial=name;
            string[] args=Environment.GetCommandLineArgs();
            int index=Array.IndexOf(args,"--crane-campus-calibration-angle-degrees");
            if(index>=0){
                if(index+1>=args.Length)throw new ArgumentException("Calibration angle requires a value");
                rotationDegrees=float.Parse(args[index+1],CultureInfo.InvariantCulture);
                if(rotationDegrees!=90&&rotationDegrees!=180)throw new ArgumentOutOfRangeException("Calibration rotation must be 90 or 180 degrees");
            }
            campus.RecordEvent("calibration-program",trial,JsonUtility.ToJson(new ProgramEvidence{trial=trial,rotationDegrees=rotationDegrees}));
        }
        void FixedUpdate(){double t=campus.Elapsed-3;float v=0,w=0;
            if(t>=0){
                switch(trial){
                    case "CAL-01":if(t<16)v=.2f;break;
                    case "CAL-02":if(t<5)v=.2f;break;
                    case "CAL-03":double turn=rotationDegrees*Math.PI/180/.4;if(t<turn)w=.4f;else if(t<turn+2)w=0;else if(t<2*turn+2)w=-.4f;break;
                    case "CAL-04":if(t<14){v=.15f;w=.2f;}break;
                    case "CAL-05":case "CAL-06":if(t<24)v=.15f;break;
                    case "CAL-07":if(t<35)v=.2f;break;
                    case "CAL-08":if(t<80){double segment=t%40;if(segment<18)v=.2f;else if(segment<20)w=.4f;else if(segment<38)v=-.2f;else w=-.4f;}break;
                    default:throw new ArgumentException(trial);
                }
            }
            drive.SetCommand(v,w);
            if(v!=lastLinear||w!=lastAngular){
                campus.RecordEvent("calibration-command",trial,JsonUtility.ToJson(new CommandEvidence{linear=v,angular=w}));
                lastLinear=v;lastAngular=w;
            }
        }
    }
}

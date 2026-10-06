using System;
using System.IO;
using UnityEngine;

namespace Sim.Physics.Land {
    /// <summary>Fixed-step repeatable commands also usable as the real-robot trial specification.</summary>
    public sealed class CraneCampusCalibration:MonoBehaviour {
        CraneIndustrialCampus campus;DifferentialDriveDynamics drive;string trial;
        public void Configure(CraneIndustrialCampus c,DifferentialDriveDynamics d,string name){campus=c;drive=d;trial=name;}
        void FixedUpdate(){double t=campus.Elapsed-3;float v=0,w=0;
            if(t>=0){
                switch(trial){
                    case "CAL-01":if(t<16)v=.2f;break;
                    case "CAL-02":if(t<5)v=.2f;break;
                    case "CAL-03":if(t<Math.PI/.4)w=.4f;else if(t<Math.PI/.4+2)w=0;else if(t<2*Math.PI/.4+2)w=-.4f;break;
                    case "CAL-04":if(t<14){v=.15f;w=.2f;}break;
                    case "CAL-05":case "CAL-06":if(t<24)v=.15f;break;
                    case "CAL-07":if(t<35)v=.2f;break;
                    case "CAL-08":if(t<80){double segment=t%40;if(segment<18)v=.2f;else if(segment<20)w=.4f;else if(segment<38)v=-.2f;else w=-.4f;}break;
                    default:throw new ArgumentException(trial);
                }
            }
            drive.SetCommand(v,w);
        }
    }
}

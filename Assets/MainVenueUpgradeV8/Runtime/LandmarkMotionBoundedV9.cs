// EXPLORATORY conditional geometric bound, not acquisition/hardware qualification.
// All arithmetic after acquisition is double precision. No physics truth input.
using System;
using System.Collections.Generic;

namespace Crane.MainVenueV8 {
    public sealed class LandmarkBoundedEstimateV9 {
        public double time_s,forward_mps,uncertainty_mps,residual_m,yaw_error_bound_rad;
        public int matched_landmarks;
        public string[] bound_pair;
    }
    public sealed class LandmarkMotionBoundedV9 {
        public const double ArithmeticPositionPadM=1e-8;
        public readonly double PositionErrorBoundM;
        Dictionary<string,double[]> previous;
        double previousTime;
        static bool Finite(double value)=>!double.IsNaN(value)&&!double.IsInfinity(value);
        static double Norm(double x,double z)=>Math.Sqrt(x*x+z*z);
        static double Up(double value)=>BitConverter.Int64BitsToDouble(BitConverter.DoubleToInt64Bits(value)+1);
        public LandmarkMotionBoundedV9(double positionErrorBoundM) {
            if(!Finite(positionErrorBoundM)||positionErrorBoundM<=0||positionErrorBoundM>=20.001)
                throw new ArgumentException("Explicit positive coordinate-error assumption required");
            PositionErrorBoundM=positionErrorBoundM;
        }
        public LandmarkBoundedEstimateV9 Observe(double time,IList<LandmarkObservationV8> observations) {
            if(!Finite(time)||time<0||observations==null||(previous!=null&&time<=previousTime))
                throw new ArgumentException("Advancing finite landmark time/observations required");
            var current=new Dictionary<string,double[]>();
            foreach(var point in observations) {
                if(point==null||string.IsNullOrWhiteSpace(point.id)||current.ContainsKey(point.id))
                    throw new ArgumentException("Unique explicit landmark identities required");
                double x=point.relative_m.x,y=point.relative_m.y,z=point.relative_m.z;
                double range=Math.Sqrt(x*x+y*y+z*z);
                if(!Finite(range)||range<=0||range>20.001)throw new ArgumentException("Finite in-range relative vectors required");
                current.Add(point.id,new[]{x,y,z});
            }
            var old=previous;double dt=time-previousTime;previous=current;previousTime=time;
            if(old==null||dt>.5)return null;
            var ids=new List<string>();foreach(var id in current.Keys)if(old.ContainsKey(id))ids.Add(id);
            ids.Sort(StringComparer.Ordinal);if(ids.Count<3)return null;
            var a=new double[3];var b=new double[3];
            foreach(var id in ids)for(int k=0;k<3;k++) {a[k]+=current[id][k];b[k]+=old[id][k];}
            for(int k=0;k<3;k++) {a[k]/=ids.Count;b[k]/=ids.Count;}
            double dot=0,cross=0,xx=0,zz=0,xz=0;
            foreach(var id in ids) {
                double cx=current[id][0]-a[0],cz=current[id][2]-a[2],px=old[id][0]-b[0],pz=old[id][2]-b[2];
                dot+=cx*px+cz*pz;cross+=cz*px-cx*pz;xx+=cx*cx;zz+=cz*cz;xz+=cx*cz;
            }
            if(xx*zz-xz*xz<1e-8||dot*dot+cross*cross<1e-12)return null;
            double yaw=Math.Atan2(cross,dot),c=Math.Cos(yaw),s=Math.Sin(yaw);
            double tx=b[0]-(c*a[0]+s*a[2]),tz=b[2]-(-s*a[0]+c*a[2]),ty=b[1]-a[1],residual=0;
            foreach(var id in ids) {
                var p=current[id];var q=old[id];
                double ex=c*p[0]+s*p[2]+tx-q[0],ey=p[1]+ty-q[1],ez=-s*p[0]+c*p[2]+tz-q[2];
                residual=Math.Max(residual,Math.Sqrt(ex*ex+ey*ey+ez*ez));
            }
            double e=PositionErrorBoundM;
            if(residual>4*e+ArithmeticPositionPadM)return null;
            double best=-1,lc=0,lp=0,pairYaw=0;string first=null,second=null;
            for(int i=0;i<ids.Count;i++)for(int j=i+1;j<ids.Count;j++) {
                var ci=current[ids[i]];var cj=current[ids[j]];var pi=old[ids[i]];var pj=old[ids[j]];
                double cx=cj[0]-ci[0],cz=cj[2]-ci[2],px=pj[0]-pi[0],pz=pj[2]-pi[2];
                double currentLength=Norm(cx,cz),previousLength=Norm(px,pz),minimum=Math.Min(currentLength,previousLength);
                if(minimum>best) {best=minimum;lc=currentLength;lp=previousLength;pairYaw=Math.Atan2(cz*px-cx*pz,cx*px+cz*pz);first=ids[i];second=ids[j];}
            }
            if(best<=2*e+ArithmeticPositionPadM)return null;
            double separation=Math.Abs(Math.Atan2(Math.Sin(yaw-pairYaw),Math.Cos(yaw-pairYaw)));
            double yawBound=Math.Min(Math.PI,separation+Math.Asin(Math.Min(1,2*e/lc))+Math.Asin(Math.Min(1,2*e/lp)));
            double pad=ArithmeticPositionPadM*(1+Norm(b[0],b[2])+Norm(a[0],a[2]));
            double uncertainty=Up((2*e+2*Norm(b[0],b[2])*Math.Sin(yawBound/2)+pad)/dt);
            double speed=(s*b[0]+c*b[2]-a[2])/dt;
            if(!Finite(speed)||!Finite(uncertainty)||!Finite(residual)||!Finite(yawBound))return null;
            return new LandmarkBoundedEstimateV9 {time_s=time,forward_mps=speed,uncertainty_mps=uncertainty,residual_m=residual,
                yaw_error_bound_rad=yawBound,matched_landmarks=ids.Count,bound_pair=new[]{first,second}};
        }
    }
}

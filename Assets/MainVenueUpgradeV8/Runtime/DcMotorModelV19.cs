// EXPLORATORY explicit ideal four-quadrant DC circuit. Not measured rover parameters.
using System;

namespace Crane.MainVenueV8 {
    [Serializable] public sealed class DcMotorDesignV19 {
        public double resistance_ohm,inductance_h,torque_constant_nm_per_a,back_emf_v_per_rad_s;
        public void Validate() {
            foreach(double value in new[]{resistance_ohm,inductance_h,torque_constant_nm_per_a,back_emf_v_per_rad_s})
                if(!CraneFaultRuntimeV8.Finite(value)||value<=0)throw new ArgumentException("Positive finite DC circuit design required");
            if(Math.Abs(torque_constant_nm_per_a-back_emf_v_per_rad_s)>1e-12)throw new ArgumentException("SI torque/back-EMF constants must agree");
        }
    }
    public sealed class DcMotorModelV19 {
        readonly DcMotorDesignV19 design;
        public double CurrentA {get;private set;}
        public double SupplyVoltageV {get;private set;}
        public double TerminalVoltageV {get;private set;}
        public double MeanCurrentA {get;private set;}
        public double MeanTorqueNm => design.torque_constant_nm_per_a*MeanCurrentA;
        public double EndTorqueNm => design.torque_constant_nm_per_a*CurrentA;
        public DcMotorModelV19(DcMotorDesignV19 parameters) {
            parameters.Validate();
            // Copy the design: a caller cannot mutate a live circuit's constants.
            design=new DcMotorDesignV19 {resistance_ohm=parameters.resistance_ohm,inductance_h=parameters.inductance_h,torque_constant_nm_per_a=parameters.torque_constant_nm_per_a,back_emf_v_per_rad_s=parameters.back_emf_v_per_rad_s};
        }
        public void Advance(double supplyVoltageV,double normalizedVoltage,double shaftRateRadS,double dt) {
            foreach(double value in new[]{supplyVoltageV,normalizedVoltage,shaftRateRadS,dt})
                if(!CraneFaultRuntimeV8.Finite(value))throw new ArgumentException("Finite electrical plant inputs required");
            if(supplyVoltageV<0||Math.Abs(normalizedVoltage)>1||dt<=0||dt>.5)throw new ArgumentException("Nonnegative supply, bounded command and step required");
            SupplyVoltageV=supplyVoltageV;TerminalVoltageV=supplyVoltageV*normalizedVoltage;
            double equilibrium=(TerminalVoltageV-design.back_emf_v_per_rad_s*shaftRateRadS)/design.resistance_ohm;
            double a=dt*design.resistance_ohm/design.inductance_h;
            double decay=Math.Exp(-a);
            // Avoid cancellation in (1-exp(-a))/a for a small positive step.
            double averageFactor=a<1e-5?1-a/2+a*a/6-a*a*a/24:(1-decay)/a;
            MeanCurrentA=equilibrium+(CurrentA-equilibrium)*averageFactor;
            CurrentA=equilibrium+(CurrentA-equilibrium)*decay;
            if(!CraneFaultRuntimeV8.Finite(CurrentA)||!CraneFaultRuntimeV8.Finite(MeanTorqueNm))throw new InvalidOperationException("Electrical state overflow");
        }
    }
}

"""Check measured injection statistics separately from navigation success."""
import json
import math


def sensor_error_evidence(events,profile):
    samples=[json.loads(e['detail']) for e in events if e['type']=='lidar-error-evidence']
    if not samples:
        return dict(checks={'measuredSensorErrorCounters':False},boundary='Missing runtime sensor injection evidence')
    final=samples[-1];n=final['noiseSamples'];hits=final['geometricReturns'];material=final['materialReturns']
    mean=final['noiseSumMeters']/max(1,n)
    std=math.sqrt(max(0,final['noiseSquaredSumMetersSquared']/max(1,n)-mean*mean))
    sigma=final['configuredSigmaMeters'];baseline=final['configuredDropoutProbability']
    material_fraction=final['materialDropouts']/max(1,material)
    checks=dict(measuredSensorErrorCounters=True,materialProfileMatchesScenario=final['materialClass']==profile,
                sufficientGeometricReturns=hits>=1000,sufficientNoiseSamples=n>=1000,
                observedInjectedDropouts=final['injectedDropouts']>0 if baseline>0 or profile!='off' else final['injectedDropouts']==0,
                monotonicSensorCounters=all(all(b[key]>=a[key] for key in ['scans','geometricReturns','injectedDropouts','materialReturns','materialDropouts','noiseSamples']) for a,b in zip(samples,samples[1:])),
                realizedNoiseScaleConsistent=(.8*sigma<=std<=1.2*sigma if sigma else std==0),
                realizedNoiseMeanConsistent=abs(mean)<=max(6*sigma/math.sqrt(max(1,n)),.0001))
    if profile in ['reflective','dark']:
        checks['materialTaggedReturnsObserved']=material>=100
        checks['materialDropoutIncreaseObserved']=material_fraction>baseline+.03
    return dict(checks=checks,counters=final,realizedRangeNoiseMeanMeters=mean,
                realizedRangeNoiseStdMeters=std,observedDropoutFraction=final['injectedDropouts']/max(1,hits),
                materialDropoutFraction=material_fraction,
                boundary='Engineering-prior error injection evidence, not measured physical LiDAR failure rates. Realized noise includes range-limit clamping.')

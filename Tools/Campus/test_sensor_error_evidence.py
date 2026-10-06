import json
from sensor_error_evidence import sensor_error_evidence


def evidence(**changes):
    values=dict(scans=100,geometricReturns=10000,injectedDropouts=300,materialReturns=1000,materialDropouts=130,
                noiseSamples=9700,noiseSumMeters=0,noiseSquaredSumMetersSquared=9700*.005**2,
                configuredSigmaMeters=.005,configuredDropoutProbability=.01,latencyScans=0,materialClass='dark')
    values.update(changes)
    return [dict(type='lidar-error-evidence',detail=json.dumps(values))]


def test_measured_dark_surface_effect_passes():
    assert all(sensor_error_evidence(evidence(),'dark')['checks'].values())


def test_ordinary_missing_ranges_do_not_prove_material_injection():
    report=sensor_error_evidence(evidence(materialReturns=0,materialDropouts=0,injectedDropouts=0),'dark')
    assert not report['checks']['materialTaggedReturnsObserved']
    assert not report['checks']['observedInjectedDropouts']


def test_wrong_noise_scale_and_wrong_profile_fail():
    report=sensor_error_evidence(evidence(noiseSquaredSumMetersSquared=0,materialClass='off'),'dark')
    assert not report['checks']['realizedNoiseScaleConsistent']
    assert not report['checks']['materialProfileMatchesScenario']


def test_missing_or_reset_counters_cannot_pass():
    assert not sensor_error_evidence([],'dark')['checks']['measuredSensorErrorCounters']
    events=evidence(scans=200)+evidence(scans=100)
    assert not sensor_error_evidence(events,'dark')['checks']['monotonicSensorCounters']

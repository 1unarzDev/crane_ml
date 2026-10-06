"""Prove wheel-level surface observations, preserving left/right differences."""
def transition_evidence(rows):
    expected=['compacted-dirt','metal-plate','rubber-mat','painted-epoxy','normal-concrete']
    counts={wheel:{} for wheel in ['leftSurface','rightSurface']}
    sequences={wheel:[] for wheel in counts}
    for row in rows:
        for wheel in counts:
            surface=row[wheel];counts[wheel][surface]=counts[wheel].get(surface,0)+1
            if not sequences[wheel] or sequences[wheel][-1]!=surface:sequences[wheel].append(surface)
    def ordered(sequence):
        index=0
        for surface in sequence:
            if index<len(expected) and surface==expected[index]:index+=1
        return index==len(expected)
    checks={wheel+'ObservedAllRequiredSurfaces':all(counts[wheel].get(surface,0)>=3 for surface in expected) for wheel in counts}
    checks.update({wheel+'ObservedOrderedTransition':ordered(sequences[wheel]) for wheel in counts})
    return dict(checks=checks,expectedSequence=expected,wheelSurfaceCounts=counts,wheelSurfaceSequences=sequences,
                boundary='Sampled environment-side wheel surface IDs; does not establish calibrated friction or measured physical slip.')

"""Measure ordered traversal of actual alternating proving-course gaps."""


def slalom_evidence(manifest, scenario, rows, footprint_radius=.24):
    boxes={b['id']:b for b in manifest['boxes']+scenario['obstacles']}
    left=boxes['prove-course-left'];right=boxes['prove-course-right']
    inner_left=left['center'][0]+left['size'][0]/2
    inner_right=right['center'][0]-right['size'][0]/2
    gates=[];checks={};cursor=0
    for index in range(3):
        bank=boxes[f'slalom-bank-{index}']; side='right' if index%2==0 else 'left'
        lo=bank['center'][0]+bank['size'][0]/2 if side=='right' else inner_left
        hi=inner_right if side=='right' else bank['center'][0]-bank['size'][0]/2
        connects=(bank['center'][0]-bank['size'][0]/2 <= inner_left if side=='right'
                  else bank['center'][0]+bank['size'][0]/2 >= inner_right)
        crossing=None
        for sample_index in range(cursor,len(rows)-1):
            a,b=rows[sample_index],rows[sample_index+1]
            za,zb=a['position']['z'],b['position']['z'];z=bank['center'][2]
            if za<=z<zb:
                alpha=(z-za)/(zb-za)
                x=a['position']['x']+alpha*(b['position']['x']-a['position']['x'])
                crossing=dict(xMeters=x,zMeters=z,simulationTime=a['simulationTime']+alpha*(b['simulationTime']-a['simulationTime']),
                              sampleGapSeconds=b['simulationTime']-a['simulationTime'])
                cursor=sample_index+1;break
        checks[f'gate{index}IsTightPhysicalGap']=.95<=hi-lo<=1.2 and connects and bank.get('yaw',0)==0
        checks[f'gate{index}MeasuredForwardCrossing']=crossing is not None
        checks[f'gate{index}CrossedInsideFootprintClearance']=crossing is not None and lo+footprint_radius<=crossing['xMeters']<=hi-footprint_radius
        checks[f'gate{index}AcquisitionGapBounded']=crossing is not None and 0<crossing['sampleGapSeconds']<=.21
        gates.append(dict(semanticId=bank['id'],side=side,clearWidthMeters=hi-lo,openingX=[lo,hi],crossing=crossing))
    return dict(checks=checks,gates=gates,footprintRadiusMeters=footprint_radius,
                boundary='Interpolated body-center crossings between acquisition samples; semantic contact evidence must separately exclude obstacle collisions. Does not measure contact impulses.')

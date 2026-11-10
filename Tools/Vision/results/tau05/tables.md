### Overall by inference mode (blur 0)


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| tiled | 1425 | 0.090 (0.08-0.11) | 0.100 (0.09-0.12) | 0.366 | 0.406 | 0.096 | 0.116 |
| full | 1425 | 0.016 (0.01-0.02) | 0.016 (0.01-0.02) | 0.958 | 0.958 | 0.038 | 0.042 |

### Apparent person height (px, 1920x1080 frame), tiled

Size/occlusion strata use COCO-style ignore: detections matched to out-of-stratum persons are dropped; unmatched detections on those images count as false positives, so precision/AP are per-image-set.

| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| <16 | 726 | 0.006 (0.00-0.01) | 0.008 (0.00-0.02) | 0.026 | 0.039 | 0.004 | 0.009 |
| 16-32 | 367 | 0.065 (0.04-0.10) | 0.082 (0.06-0.11) | 0.348 | 0.435 | 0.078 | 0.110 |
| 32-64 | 181 | 0.204 (0.15-0.27) | 0.221 (0.17-0.29) | 0.725 | 0.784 | 0.273 | 0.300 |
| 64-128 | 151 | 0.417 (0.34-0.50) | 0.437 (0.36-0.52) | 0.818 | 0.857 | 0.514 | 0.589 |
| >=128 | 0 | nan (nan-nan) | nan (nan-nan) | nan | nan | nan | nan |

### Apparent person height, full-frame 640 letterbox

Size/occlusion strata use COCO-style ignore: detections matched to out-of-stratum persons are dropped; unmatched detections on those images count as false positives, so precision/AP are per-image-set.

| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| <16 | 726 | 0.000 (0.00-0.01) | 0.000 (0.00-0.01) | 0.000 | 0.000 | 0.000 | 0.000 |
| 16-32 | 367 | 0.000 (0.00-0.01) | 0.000 (0.00-0.01) | nan | nan | 0.012 | 0.016 |
| 32-64 | 181 | 0.033 (0.02-0.07) | 0.033 (0.02-0.07) | 1.000 | 1.000 | 0.109 | 0.112 |
| 64-128 | 151 | 0.113 (0.07-0.17) | 0.113 (0.07-0.17) | 1.000 | 1.000 | 0.249 | 0.293 |
| >=128 | 0 | nan (nan-nan) | nan (nan-nan) | nan | nan | nan | nan |

### Occlusion (NOMAD visibility label), tiled

Size/occlusion strata use COCO-style ignore: detections matched to out-of-stratum persons are dropped; unmatched detections on those images count as false positives, so precision/AP are per-image-set.

| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| 10-30% | 630 | 0.025 (0.02-0.04) | 0.032 (0.02-0.05) | 0.147 | 0.183 | 0.018 | 0.025 |
| 40-60% | 559 | 0.120 (0.10-0.15) | 0.131 (0.11-0.16) | 0.435 | 0.474 | 0.133 | 0.162 |
| 70-90% | 213 | 0.192 (0.15-0.25) | 0.211 (0.16-0.27) | 0.461 | 0.506 | 0.251 | 0.281 |
| 100% | 23 | 0.174 (0.07-0.37) | 0.174 (0.07-0.37) | 0.667 | 0.667 | 0.503 | 0.507 |

### NOMAD distance stratum (m), tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| 10 | 284 | 0.324 (0.27-0.38) | 0.349 (0.30-0.41) | 0.736 | 0.792 | 0.391 | 0.453 |
| 30 | 288 | 0.080 (0.05-0.12) | 0.097 (0.07-0.14) | 0.489 | 0.596 | 0.123 | 0.168 |
| 50 | 292 | 0.041 (0.02-0.07) | 0.048 (0.03-0.08) | 0.197 | 0.230 | 0.030 | 0.045 |
| 70 | 276 | 0.000 (0.00-0.01) | 0.000 (0.00-0.01) | 0.000 | 0.000 | 0.005 | 0.007 |
| 90 | 285 | 0.004 (0.00-0.02) | 0.004 (0.00-0.02) | 0.020 | 0.020 | 0.003 | 0.004 |

### Lighting: sky, tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| sunny | 741 | 0.072 (0.06-0.09) | 0.074 (0.06-0.10) | 0.225 | 0.233 | 0.072 | 0.080 |
| partcloud | 360 | 0.125 (0.09-0.16) | 0.142 (0.11-0.18) | 0.625 | 0.708 | 0.148 | 0.202 |
| overcast | 324 | 0.093 (0.07-0.13) | 0.111 (0.08-0.15) | 0.714 | 0.857 | 0.164 | 0.205 |

### Lighting: sun height (low = before 10:00 or from 17:00), tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| high_sun | 981 | 0.073 (0.06-0.09) | 0.078 (0.06-0.10) | 0.298 | 0.318 | 0.078 | 0.088 |
| low_sun | 444 | 0.126 (0.10-0.16) | 0.146 (0.12-0.18) | 0.519 | 0.602 | 0.143 | 0.192 |

### Activity (pose), tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| Walking | 37 | 0.432 (0.29-0.59) | 0.432 (0.29-0.59) | 0.640 | 0.640 | 0.488 | 0.499 |
| Hiding | 1028 | 0.073 (0.06-0.09) | 0.085 (0.07-0.10) | 0.300 | 0.348 | 0.075 | 0.095 |
| Laying | 16 | 0.375 (0.18-0.61) | 0.375 (0.18-0.61) | 1.000 | 1.000 | 0.981 | 0.981 |
| Hiding (Laying) | 344 | 0.090 (0.06-0.13) | 0.096 (0.07-0.13) | 0.449 | 0.478 | 0.110 | 0.125 |

### Scene (location), tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| Farm | 180 | 0.089 (0.06-0.14) | 0.089 (0.06-0.14) | 0.267 | 0.267 | 0.094 | 0.098 |
| Field | 120 | 0.200 (0.14-0.28) | 0.208 (0.15-0.29) | 0.632 | 0.658 | 0.225 | 0.281 |
| Forest | 180 | 0.044 (0.02-0.09) | 0.056 (0.03-0.10) | 0.533 | 0.667 | 0.075 | 0.129 |
| Golf Course | 180 | 0.056 (0.03-0.10) | 0.056 (0.03-0.10) | 0.769 | 0.769 | 0.090 | 0.092 |
| Lake | 24 | 0.208 (0.09-0.40) | 0.417 (0.24-0.61) | 0.417 | 0.833 | 0.270 | 0.667 |
| Paintball | 180 | 0.094 (0.06-0.15) | 0.094 (0.06-0.15) | 0.230 | 0.230 | 0.070 | 0.073 |
| Quarry | 180 | 0.200 (0.15-0.26) | 0.228 (0.17-0.29) | 0.783 | 0.891 | 0.349 | 0.407 |
| School | 201 | 0.025 (0.01-0.06) | 0.030 (0.01-0.06) | 0.065 | 0.078 | 0.019 | 0.024 |
| School (Nature) | 180 | 0.039 (0.02-0.08) | 0.039 (0.02-0.08) | 0.467 | 0.467 | 0.044 | 0.049 |

### Synthetic horizontal motion blur (kernel length px), tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| 0 px | 1425 | 0.090 (0.08-0.11) | 0.100 (0.09-0.12) | 0.366 | 0.406 | 0.096 | 0.116 |
| 3 px | 1425 | 0.079 (0.07-0.09) | 0.086 (0.07-0.10) | 0.351 | 0.382 | 0.080 | 0.098 |
| 6 px | 1425 | 0.060 (0.05-0.07) | 0.066 (0.05-0.08) | 0.248 | 0.271 | 0.055 | 0.066 |
| 12 px | 1425 | 0.036 (0.03-0.05) | 0.041 (0.03-0.05) | 0.173 | 0.201 | 0.025 | 0.030 |
| 24 px | 1425 | 0.013 (0.01-0.02) | 0.015 (0.01-0.02) | 0.084 | 0.098 | 0.007 | 0.009 |

### Recall (loc) by blur x apparent height, tiled

| blur px | <16 | 16-32 | 32-64 | 64-128 | >=128 |
|---|---:|---:|---:|---:|---:|
| 0 | 0.008 | 0.082 | 0.221 | 0.437 | nan |
| 3 | 0.004 | 0.057 | 0.232 | 0.377 | nan |
| 6 | 0.003 | 0.033 | 0.177 | 0.318 | nan |
| 12 | 0.001 | 0.016 | 0.105 | 0.219 | nan |
| 24 | 0.000 | 0.000 | 0.022 | 0.113 | nan |

### False alerts (detections not on the annotated person), blur 0

Footprint is 1920x1080 px times the squared GSD at the person (NOMAD paper Table S3, scaled to 1920 px); oblique ~45 deg views make the true footprint larger, so FP/km^2 is an upper bound.

| mode | distance m | tau | frames | FP/frame | frames with >=1 FP | approx footprint km^2 | FP/km^2 |
|---|---:|---:|---:|---:|---:|---:|---:|
| tiled | 10 | 0.3 | 284 | 0.479 | 0.296 | 0.00030 | 1582 |
| tiled | 10 | 0.5 | 284 | 0.063 | 0.053 | 0.00030 | 209 |
| tiled | 30 | 0.3 | 288 | 0.663 | 0.285 | 0.00273 | 243 |
| tiled | 30 | 0.5 | 288 | 0.062 | 0.062 | 0.00273 | 23 |
| tiled | 50 | 0.3 | 280 | 0.700 | 0.400 | 0.00757 | 92 |
| tiled | 50 | 0.5 | 280 | 0.168 | 0.136 | 0.00757 | 22 |
| tiled | 70 | 0.3 | 276 | 0.826 | 0.486 | 0.01484 | 56 |
| tiled | 70 | 0.5 | 276 | 0.239 | 0.207 | 0.01484 | 16 |
| tiled | 90 | 0.3 | 276 | 0.725 | 0.420 | 0.02453 | 30 |
| tiled | 90 | 0.5 | 276 | 0.181 | 0.145 | 0.02453 | 7 |
| full | 10 | 0.3 | 284 | 0.151 | 0.077 | 0.00030 | 500 |
| full | 10 | 0.5 | 284 | 0.000 | 0.000 | 0.00030 | 0 |
| full | 30 | 0.3 | 288 | 0.198 | 0.094 | 0.00273 | 73 |
| full | 30 | 0.5 | 288 | 0.000 | 0.000 | 0.00273 | 0 |
| full | 50 | 0.3 | 280 | 0.075 | 0.061 | 0.00757 | 10 |
| full | 50 | 0.5 | 280 | 0.004 | 0.004 | 0.00757 | 0 |
| full | 70 | 0.3 | 276 | 0.080 | 0.058 | 0.01484 | 5 |
| full | 70 | 0.5 | 276 | 0.000 | 0.000 | 0.01484 | 0 |
| full | 90 | 0.3 | 276 | 0.116 | 0.087 | 0.02453 | 5 |
| full | 90 | 0.5 | 276 | 0.000 | 0.000 | 0.02453 | 0 |

FP persistence: of 150 false alerts (tau 0.5, tiled), 0.807 reappear within 40 px in the next annotated frame (<=12 video frames later): background false alerts are mostly persistent decoys, not white noise.

### Repeat-view correlation (tiled, blur 0, loc criterion, tau 0.50)

| quantity | value |
|---|---:|
| consecutive annotated-frame pairs (<=12 video frames, 0.4 s at 30 fps) | 1053 |
| runs | 351 |
| P(detect) marginal | 0.100 |
| P(detect at t+1 \| detect at t) | 0.718 |
| P(detect at t+1 \| miss at t) | 0.029 |
| ICC within run (binary ANOVA) | 0.625 |
| lag-1 residual correlation after height x visibility cells | 0.611 |
| ICC of run means within a flight (different minutes, poses) | 0.495 |

### Box-centre error of matched detections (px, 1920 frame), tiled

| height | n | RMS x | RMS y | median radial |
|---|---:|---:|---:|---:|
| <16 | 5 | 3.3 | 2.0 | 2.5 |
| 16-32 | 29 | 1.6 | 2.5 | 1.5 |
| 32-64 | 40 | 3.6 | 3.5 | 3.0 |
| 64-128 | 66 | 6.2 | 6.6 | 2.6 |

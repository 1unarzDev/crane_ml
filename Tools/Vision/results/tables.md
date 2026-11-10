### Overall by inference mode (blur 0)


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| tiled | 1425 | 0.147 (0.13-0.17) | 0.172 (0.15-0.19) | 0.171 | 0.199 | 0.096 | 0.116 |
| full | 1425 | 0.037 (0.03-0.05) | 0.041 (0.03-0.05) | 0.226 | 0.248 | 0.038 | 0.042 |

### Apparent person height (px, 1920x1080 frame), tiled

Size/occlusion strata use COCO-style ignore: detections matched to out-of-stratum persons are dropped; unmatched detections on those images count as false positives, so precision/AP are per-image-set.

| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| <16 | 726 | 0.034 (0.02-0.05) | 0.055 (0.04-0.07) | 0.043 | 0.070 | 0.004 | 0.009 |
| 16-32 | 367 | 0.134 (0.10-0.17) | 0.166 (0.13-0.21) | 0.145 | 0.180 | 0.078 | 0.110 |
| 32-64 | 181 | 0.304 (0.24-0.37) | 0.309 (0.25-0.38) | 0.307 | 0.313 | 0.273 | 0.300 |
| 64-128 | 151 | 0.536 (0.46-0.61) | 0.583 (0.50-0.66) | 0.551 | 0.599 | 0.514 | 0.589 |
| >=128 | 0 | nan (nan-nan) | nan (nan-nan) | nan | nan | nan | nan |

### Apparent person height, full-frame 640 letterbox

Size/occlusion strata use COCO-style ignore: detections matched to out-of-stratum persons are dropped; unmatched detections on those images count as false positives, so precision/AP are per-image-set.

| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| <16 | 726 | 0.000 (0.00-0.01) | 0.000 (0.00-0.01) | 0.000 | 0.000 | 0.000 | 0.000 |
| 16-32 | 367 | 0.005 (0.00-0.02) | 0.011 (0.00-0.03) | 0.080 | 0.160 | 0.012 | 0.016 |
| 32-64 | 181 | 0.105 (0.07-0.16) | 0.110 (0.07-0.16) | 0.306 | 0.323 | 0.109 | 0.112 |
| 64-128 | 151 | 0.212 (0.15-0.28) | 0.225 (0.17-0.30) | 0.492 | 0.523 | 0.249 | 0.293 |
| >=128 | 0 | nan (nan-nan) | nan (nan-nan) | nan | nan | nan | nan |

### Occlusion (NOMAD visibility label), tiled

Size/occlusion strata use COCO-style ignore: detections matched to out-of-stratum persons are dropped; unmatched detections on those images count as false positives, so precision/AP are per-image-set.

| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| 10-30% | 630 | 0.051 (0.04-0.07) | 0.075 (0.06-0.10) | 0.067 | 0.098 | 0.018 | 0.025 |
| 40-60% | 559 | 0.181 (0.15-0.21) | 0.209 (0.18-0.24) | 0.196 | 0.227 | 0.133 | 0.162 |
| 70-90% | 213 | 0.310 (0.25-0.37) | 0.329 (0.27-0.39) | 0.277 | 0.294 | 0.251 | 0.281 |
| 100% | 23 | 0.478 (0.29-0.67) | 0.478 (0.29-0.67) | 0.423 | 0.423 | 0.503 | 0.507 |

### NOMAD distance stratum (m), tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| 10 | 284 | 0.419 (0.36-0.48) | 0.461 (0.40-0.52) | 0.405 | 0.446 | 0.391 | 0.453 |
| 30 | 288 | 0.174 (0.13-0.22) | 0.215 (0.17-0.27) | 0.193 | 0.239 | 0.123 | 0.168 |
| 50 | 292 | 0.079 (0.05-0.12) | 0.113 (0.08-0.15) | 0.100 | 0.144 | 0.030 | 0.045 |
| 70 | 276 | 0.040 (0.02-0.07) | 0.043 (0.03-0.07) | 0.046 | 0.050 | 0.005 | 0.007 |
| 90 | 285 | 0.025 (0.01-0.05) | 0.025 (0.01-0.05) | 0.034 | 0.034 | 0.003 | 0.004 |

### Lighting: sky, tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| sunny | 741 | 0.117 (0.10-0.14) | 0.128 (0.11-0.15) | 0.108 | 0.118 | 0.072 | 0.080 |
| partcloud | 360 | 0.186 (0.15-0.23) | 0.233 (0.19-0.28) | 0.246 | 0.309 | 0.148 | 0.202 |
| overcast | 324 | 0.173 (0.14-0.22) | 0.204 (0.16-0.25) | 0.371 | 0.437 | 0.164 | 0.205 |

### Lighting: sun height (low = before 10:00 or from 17:00), tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| high_sun | 981 | 0.126 (0.11-0.15) | 0.136 (0.12-0.16) | 0.160 | 0.171 | 0.078 | 0.088 |
| low_sun | 444 | 0.194 (0.16-0.23) | 0.252 (0.21-0.29) | 0.189 | 0.247 | 0.143 | 0.192 |

### Activity (pose), tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| Walking | 37 | 0.514 (0.36-0.67) | 0.514 (0.36-0.67) | 0.404 | 0.404 | 0.488 | 0.499 |
| Hiding | 1028 | 0.135 (0.12-0.16) | 0.164 (0.14-0.19) | 0.151 | 0.183 | 0.075 | 0.095 |
| Laying | 16 | 0.625 (0.39-0.82) | 0.625 (0.39-0.82) | 1.000 | 1.000 | 0.981 | 0.981 |
| Hiding (Laying) | 344 | 0.122 (0.09-0.16) | 0.137 (0.10-0.18) | 0.168 | 0.188 | 0.110 | 0.125 |

### Scene (location), tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| Farm | 180 | 0.139 (0.10-0.20) | 0.144 (0.10-0.20) | 0.155 | 0.161 | 0.094 | 0.098 |
| Field | 120 | 0.242 (0.17-0.33) | 0.317 (0.24-0.40) | 0.197 | 0.259 | 0.225 | 0.281 |
| Forest | 180 | 0.094 (0.06-0.15) | 0.128 (0.09-0.18) | 0.254 | 0.343 | 0.075 | 0.129 |
| Golf Course | 180 | 0.100 (0.06-0.15) | 0.100 (0.06-0.15) | 0.173 | 0.173 | 0.090 | 0.092 |
| Lake | 24 | 0.375 (0.21-0.57) | 0.792 (0.60-0.91) | 0.184 | 0.388 | 0.270 | 0.667 |
| Paintball | 180 | 0.122 (0.08-0.18) | 0.128 (0.09-0.18) | 0.097 | 0.102 | 0.070 | 0.073 |
| Quarry | 180 | 0.361 (0.29-0.43) | 0.389 (0.32-0.46) | 0.411 | 0.443 | 0.349 | 0.407 |
| School | 201 | 0.075 (0.05-0.12) | 0.090 (0.06-0.14) | 0.054 | 0.065 | 0.019 | 0.024 |
| School (Nature) | 180 | 0.056 (0.03-0.10) | 0.056 (0.03-0.10) | 0.244 | 0.244 | 0.044 | 0.049 |

### Synthetic horizontal motion blur (kernel length px), tiled


| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |
|---|---:|---|---|---:|---:|---:|---:|
| 0 px | 1425 | 0.147 (0.13-0.17) | 0.172 (0.15-0.19) | 0.171 | 0.199 | 0.096 | 0.116 |
| 3 px | 1425 | 0.124 (0.11-0.14) | 0.146 (0.13-0.17) | 0.114 | 0.134 | 0.080 | 0.098 |
| 6 px | 1425 | 0.095 (0.08-0.11) | 0.110 (0.09-0.13) | 0.062 | 0.072 | 0.055 | 0.066 |
| 12 px | 1425 | 0.061 (0.05-0.07) | 0.078 (0.07-0.09) | 0.032 | 0.041 | 0.025 | 0.030 |
| 24 px | 1425 | 0.029 (0.02-0.04) | 0.035 (0.03-0.05) | 0.015 | 0.018 | 0.007 | 0.009 |

### Recall (loc) by blur x apparent height, tiled

| blur px | <16 | 16-32 | 32-64 | 64-128 | >=128 |
|---|---:|---:|---:|---:|---:|
| 0 | 0.055 | 0.166 | 0.309 | 0.583 | nan |
| 3 | 0.039 | 0.125 | 0.293 | 0.536 | nan |
| 6 | 0.014 | 0.082 | 0.260 | 0.464 | nan |
| 12 | 0.008 | 0.044 | 0.199 | 0.351 | nan |
| 24 | 0.001 | 0.011 | 0.094 | 0.185 | nan |

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

FP persistence: of 724 false alerts (tau 0.3, tiled), 0.805 reappear within 40 px in the next annotated frame (<=12 video frames later): background false alerts are mostly persistent decoys, not white noise.

### Repeat-view correlation (tiled, blur 0, loc criterion, tau 0.30)

| quantity | value |
|---|---:|
| consecutive annotated-frame pairs (<=12 video frames, 0.4 s at 30 fps) | 1053 |
| runs | 351 |
| P(detect) marginal | 0.170 |
| P(detect at t+1 \| detect at t) | 0.751 |
| P(detect at t+1 \| miss at t) | 0.054 |
| ICC within run (binary ANOVA) | 0.596 |
| lag-1 residual correlation after height x visibility cells | 0.595 |
| ICC of run means within a flight (different minutes, poses) | 0.643 |

### Box-centre error of matched detections (px, 1920 frame), tiled

| height | n | RMS x | RMS y | median radial |
|---|---:|---:|---:|---:|
| <16 | 36 | 2.6 | 2.7 | 2.3 |
| 16-32 | 59 | 2.2 | 3.3 | 1.8 |
| 32-64 | 56 | 3.3 | 3.2 | 3.0 |
| 64-128 | 88 | 6.2 | 7.3 | 3.0 |

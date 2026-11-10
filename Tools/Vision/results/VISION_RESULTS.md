# M4: onboard person-detector benchmark and calibrated observation model

Date: 2026-11-09. Owner: M4 vision agent. Evidence label: **HSUR** (real aerial imagery, simulated motion blur, desktop GPU).
None of these numbers come from CRANE renders. The rendered rerun is specified in `RERUN_PLAN.md`. Only a 20-frame smoke clip has been scored: a fern decoy fired in 17 of 20 person-free forest frames at tau 0.5.
Figures: `recall_strata.png/.pdf` and `calibration.png/.pdf` (tau 0.3), and the same files under `tau05/`. The PDFs use the Type 1 core font Helvetica (`pdf.use14corefonts`), which is not embedded. Embed it with `gs -dEmbedAllFonts=true` if a venue requires that.
Code: `Tools/Vision/`. The literature review is in `Docs/DroneVisuals/mission/VISION_REVIEW.md`.

## 1. Choices and licenses

| item | choice | license / terms |
|---|---|---|
| detector | RT-DETR-R18 (`PekingU/rtdetr_r18vd`, HF sha ac77a11f), COCO-pretrained, class `person`, **not fine-tuned** | **Apache-2.0** (model card). Chosen over Ultralytics YOLO11 (AGPL-3.0) so that no copyleft obligation reaches the onboard stack |
| runtime | PyTorch 2.5.1+cu124, transformers 4.46.3, eager (no TensorRT) | BSD / Apache-2.0 |
| dataset | NOMAD (Russell Bernal et al., WACV 2024): 1,404 frames, 24 actors, 9 sites | **CC BY 4.0**: attribution required, commercial use allowed |
| storage | weights in `~/.cache/research/vision-data/weights_rtdetr_r18vd/`, frames in `~/.cache/research/vision-data/nomad_subset/` | never committed |

Why NOMAD:
- It is the only candidate that met every requirement and downloaded without a login.
- It has per-box occlusion labels (10 visibility levels), five distances (10/30/50/70/90 m, about 45 deg look-down), and site, weather and time metadata.
- Its consecutive annotated frames make repeat-view correlation measurable.
- Its actor, flight and site IDs allow leakage-safe splits.

Datasets that did not work out:
- **HERIDAL** (CC BY 3.0): the host was unreachable.
- **SARD**: needs a login or account.
- **WiSARD** (MIT): about 40 GB, deferred.
- **Okutama** (license unstated, and the only labelled nadir/oblique contrast): the Dropbox listing failed.
- **VisDrone** (CC BY-NC-SA): urban scenes. Mirrors are available if needed.
- **ForestPersons** (CC BY-NC-SA 4.0, gated): 1.5–2 m camera height. It is the right next dataset for under-canopy views, but its license is non-commercial and extends to trained models.

**Split.** Splits are by *site*, so no actor, flight or site is shared:
- fit: Paintball, School, Farm, Golf Course (12 actors, 720 frames)
- held-out: Forest, School (Nature), Quarry, Field, Lake (12 actors, 684 frames)

Frames are three runs of four consecutive annotated frames per actor and distance, spaced at most 12 video frames (0.4 s) apart. They were downsampled from 5472x3078 to the 1920x1080 OAK-D S2 RGB stream that CRANE renders (`RenderProfiles.OakD`).

**Inference modes on the 1080p frame:**
- `tiled`: six native-resolution 640x640 tiles, batched, merged with NMS.
- `full`: letterboxed to 640.

**Match criteria:**
- `IoU.5`: COCO.
- `loc`: IoU >= 0.25 or the detection centre falls inside the box. This is the SAR-relevant "alert lands on the person" test, and the observation model is fitted to it.

## 2. Benchmark (tiled unless stated; full tables in `tables.md`, threshold 0.5 in `tau05/tables.md`)

**Overall (blur 0, n = 1,425 persons):**

| mode | tau | recall loc | precision loc | AP50 | AP loc | FP / frame |
|---|---:|---:|---:|---:|---:|---:|
| tiled | 0.3 | 0.172 (0.15–0.19) | 0.199 | 0.096 | 0.116 | 0.68 |
| tiled | 0.5 | 0.100 (0.09–0.12) | 0.406 | 0.096 | 0.116 | 0.14 |
| full 640 | 0.3 | 0.041 (0.03–0.05) | 0.248 | 0.038 | 0.042 | 0.12 |

Tiling quadruples recall at a cost of 6 tiles per frame. A COCO-pretrained detector without aerial fine-tuning is weak on NOMAD's mostly hiding and occluded persons; the NOMAD authors report the same pattern for COCO-only YOLOv8, Faster R-CNN and RetinaNet.

**By apparent person height** (px in the 1080p frame), recall loc at tau 0.3 / 0.5:

| height | n | tiled tau 0.3 | tiled tau 0.5 | full tau 0.3 | AP50 tiled |
|---|---:|---:|---:|---:|---:|
| < 16 | 726 | 0.055 | 0.008 | 0.000 | 0.004 |
| 16–32 | 367 | 0.166 | 0.082 | 0.011 | 0.078 |
| 32–64 | 181 | 0.309 | 0.221 | 0.110 | 0.273 |
| 64–128 | 151 | 0.583 | 0.437 | 0.225 | 0.514 |

**By occlusion** (NOMAD visibility), tau 0.3:

| visible | 10–30 % | 40–60 % | 70–90 % | 100 % |
|---|---:|---:|---:|---:|
| n | 630 | 559 | 213 | 23 |
| recall loc | 0.075 | 0.209 | 0.329 | 0.478 |

**By distance** (about 45 deg look-down; slant range is about 1.4x the distance), tau 0.3:

| distance m | 10 | 30 | 50 | 70 | 90 |
|---|---:|---:|---:|---:|---:|
| recall loc | 0.461 | 0.215 | 0.113 | 0.043 | 0.025 |
| AP50 | 0.391 | 0.123 | 0.030 | 0.005 | 0.003 |

**Synthetic motion blur.** A horizontal box kernel of L px stands for f·v·t_exp/R + f·ω·t_exp; for example, 6 px is 4.5 m/s at 10 m range with a 1/100 s exposure. Recall loc at tau 0.3 by height:

| blur px | all | 16–32 | 32–64 | 64–128 |
|---:|---:|---:|---:|---:|
| 0 | 0.172 | 0.166 | 0.309 | 0.583 |
| 3 | 0.146 | 0.125 | 0.293 | 0.536 |
| 6 | 0.110 | 0.082 | 0.260 | 0.464 |
| 12 | 0.078 | 0.044 | 0.199 | 0.351 |
| 24 | 0.035 | 0.011 | 0.094 | 0.185 |

Blur also *raises* false alerts at tau 0.3, from 0.68 to 1.97 per frame between 0 and 24 px. At tau 0.5 the rate stays flat at 0.14–0.18.

**Lighting.**
- Sky: sunny 0.128, partly cloudy 0.233, overcast 0.204.
- Sun: high 0.136, low (before 10:00 or from 17:00) 0.252.

Both are **confounded with site** (only 9 sites), so the fitted lighting coefficients should not be read as causal. By site, recall ranges from 0.06 (School (Nature)) to 0.39 (Quarry) and 0.79 (Lake, n = 24).

**Viewpoint.** NOMAD is oblique only, at roughly 45 deg look-down; the gimbal pitch is not reported. **Nadir versus oblique was not measured.** In the model, view angle acts only through apparent size, plus an uncalibrated PRIOR factor (`view_prior`). The renders in `RERUN_PLAN.md` (top_30m, top_50m) and Okutama, which has 45/90 deg labels, are the planned sources.

**False alerts.** These are detections that do not land on the annotated actor. They are an upper bound, because NOMAD annotates only the actor and any bystanders would count.

| | tau 0.3 | tau 0.5 |
|---|---:|---:|
| FP / frame (tiled) | 0.48 (10 m) to 0.83 (70 m) | 0.06 (10–30 m) to 0.24 (70 m) |
| FP / km^2 of imaged ground (NOMAD GSD, nominal footprint) | ~1.6k at 10 m, ~30 at 90 m | 209 at 10 m, 7 at 90 m |
| frames with >= 1 FP | 0.29–0.49 | 0.05–0.21 |

**80 % of false alerts reappear** within 40 px in the next frame. They are persistent decoys, such as person-like objects, not white noise; this matches the placeholder's decoy concept.

**Repeat-view correlation** (tiled, blur 0, loc; frame pairs at most 0.4 s apart):

| | tau 0.3 | tau 0.5 |
|---|---:|---:|
| P(detect) | 0.170 | 0.100 |
| P(detect at t+1 \| detect at t) | 0.751 | 0.718 |
| P(detect at t+1 \| miss at t) | 0.054 | 0.029 |
| ICC within a run | 0.596 | 0.625 |
| residual lag-1 correlation given height x visibility | 0.595 | 0.611 |
| ICC of run means within a flight (minutes apart, other poses) | 0.643 | 0.495 |

Detection outcomes are strongly correlated, and not only frame to frame: the correlation persists across a flight. A miss now predicts a miss later from the same viewpoint. Re-looking from the same view buys little: with rho about 0.6, n_eff(10 frames) is about 1.6. A *different* view (another distance, another drone, another pose) is what decorrelates.

**Localization.** The box-centre error of matched detections is 2–3 px RMS per axis for persons under 64 px tall and 6–7 px for 64–128 px.

## 3. Latency (RTX 4070 Ti SUPER, shared host; `latency_raw.json`)

End to end means numpy 1080p frame to boxes on the host, including H2D copy, tiling, forward, NMS and D2H; 600 frames per cell.

The navigation load ran in separate processes:
- 4 CPU processes voxelizing 120k-point clouds at 0.1 m (about 300 clouds/s in total).
- 1 GPU process voxelizing 300k points into a 256^3 grid plus a 3^3 dilation at 20 Hz.

**The GPU and CPUs were shared with other sessions' Unity and sim jobs.** "Idle" whole-GPU utilization averaged 16–83 %, so idle and loaded cells are not cleanly separated.

| config | load | p50 ms | p95 ms | p99 ms |
|---|---|---:|---:|---:|
| tiled 6x640 fp16 | idle | 20.8 | 36.2 | 38.9 |
| tiled 6x640 fp16 | nav | 22.3 | 39.4 | 51.9 |
| tiled 6x640 fp32 | idle | 31.0 | 38.6 | 45.2 |
| tiled 6x640 fp32 | nav | 36.1 | 46.5 | 68.8 |
| full 640 fp16 | idle | 28.8 | 35.7 | 43.7 |
| full 640 fp16 | nav | 17.1 | 28.5 | 39.1 |
| forward only, batch 6, fp16 | nav | 18.0 | 19.8 | 21.0 |
| forward only, batch 1, fp16 | nav | 15.3 | 17.4 | 17.7 |

Batch 1 is launch- and Python-overhead bound: 14 ms for a single 640 tile, against about 4.6 ms with TensorRT on a T4 per the model card. That is why six tiles cost barely more than one. The navigation load mainly fattens the tail: tiled fp16 p99 rises from 39 to 52 ms.

**Jetson Orin Nano estimate (an ESTIMATE, not measured).** It uses published TensorRT FP16 numbers as cited in VISION_REVIEW.md:
- RT-DETR-R18 on Orin Nano Super at 25 W: 72.7 FPS, p95 13.8 ms (arXiv 2608.00927).
- Orin Nano, variant unstated: 81 FPS (arXiv 2606.14005).

Scaling:
- Engine-only latency is compute bound, so scale it linearly with tiles. For **tiled 6x640**: about 6 x 12.3–13.8 ms = 74–83 ms engine time, plus 5–10 ms pre/post on the ARM cores, giving **p50 about 80–95 ms (about 11 Hz)**.
- Apply the tail inflation measured here under navigation load (p95/p50 = 1.8, p99/p50 = 2.3) for **p95 about 140–170 ms and p99 about 185–220 ms**.
- Non-Super clocks add roughly another 1.6x.
- **Full 640** costs about 13–15 ms plus 5 ms, so about 50 Hz, but with a quarter of the recall.

Recommendation for the mission sim: model detections at **5–10 Hz with tiling**. NavPilot and the placeholder assume 10 Hz, which is optimistic for tiling on Orin Nano.

## 4. Calibrated observation model (`Tools/Vision/obs_model.py`, parameters in `obs_params_tau05.json` / `obs_params_tau03.json`)

**Per-frame detection.** The model is fitted to the loc criterion, tiled mode, with score >= tau:

`p = sigmoid(lp) * sigmoid(b0 + b_h*(log2 h - 5) + b_vis*(vis - 1) + b_blur*log1p(blur/h) + b_oc*overcast + b_ls*low_sun) * view_prior(view)`

Final model = pooled fit on all 9 sites. Coefficients:

| tau | b0 | b_h | b_vis | b_blur | b_oc | b_ls |
|---|---:|---:|---:|---:|---:|---:|
| 0.5 (default) | -2.05 | 1.20 | 1.30 | -1.56 | 0.24 | 0.87 |
| 0.3 | -1.04 | 0.94 | 1.53 | -2.00 | 0.36 | 1.01 |

The lighting terms are site-confounded.

**Validation by leave-one-site-out** (9 folds; each site predicted by a model fitted on the other 8). Each cell is log loss / Brier / ECE:

| tau | final model | marginal rate | MissionSim placeholder curve |
|---|---|---|---|
| 0.5 | 0.161 / 0.047 / 0.009 | 0.237 / 0.058 / 0.001 | 0.173 / 0.043 / 0.009 |
| 0.3 | 0.255 / 0.077 / 0.014 | 0.352 / 0.098 / 0.064 | 0.342 / 0.074 / 0.046 |

The reliability diagrams are in `calibration.png` / `.pdf` and `tau05/`.

**Validation on the pre-declared 4-site to 5-site split.** This one is *not* well calibrated. At tau 0.3 the ECE is 0.096: the model predicts 0.07 where 0.15 was observed, because the held-out sites (Quarry, Field, Lake) are easier. At tau 0.5 the log loss is 0.249, against 0.305 for the marginal rate and 0.232 for the placeholder.

The cause is large between-site heterogeneity: a random logit offset with sigma_scene = 1.43 at tau 0.3 and 1.19 at tau 0.5. The model therefore offers:
- `ObsModel()`: the population average, which is calibrated across sites.
- `ObsModel.draw_scene(rng)`: one site's offset, for the truth side of a mission.

Use both: the planner gets the average and the world gets a drawn site.

Other model components:
- **Confidence.** Scores of true detections follow u = (s - tau)/(1 - tau) ~ Beta(a0 + a1·q, b), where q is the per-frame p. At tau 0.5, a0 = 0.76, a1 = 2.79, b = 2.28. False alerts follow Beta(1.00, 4.07), so a likelihood-ratio threshold on the score is informative (`log_lr`).
- **False alerts.** Per frame, interpolated by blur; converted to per m^2 at the mission GSD (`false_alarm_rate_m2`); 80 % persistent (`p_decoy_detect`).
- **Repeat views.** Beta-binomial with rho = 0.61 (`p_any(p, n)`, `n_eff`). This replaces the placeholder's N_EFF = 3 cap.
- **Localization.** `loc_error(alt, ground_range, h_px)` is the flat-ground ray projection of the pixel-centre error (measured: sigma_px = 0.84 + 0.061·h) plus PRIOR attitude error (0.5 deg tilt, 1 deg yaw) and height error (0.3 m). The test checks it against Monte Carlo within 10 %. Example values:
  - 30 m altitude, 40 m ground range: 0.85 m along-range x 0.71 m cross-range.
  - 3 m altitude under canopy, 10 m range: about 1.1 x 0.2 m.
- **MissionSim drop-in.** `missionsim.obs_model.load(prefer_vision=True)` returns `ObsModel`, with the placeholder's signatures checked by test. Effects on the sim's calls at the default tau 0.5, placeholder vs vision:
  - Per-frame P for a lying person (3 m altitude, 3 m/s) is 0.65 at 4 m, 0.34 at 8 m, 0.18 at 12 m and 0.07 at 20 m.
  - Scan-pass detection probability is 0.80–0.96 (placeholder 0.32–0.99). Passes contain many frames, so blur and dense stands hurt less than in the placeholder, but the correlation caps the gain from more frames.
  - Review-look detection is 0.86–0.97.
  - Background false alerts per 400 m^2 saturate the placeholder's 0.5 cap (placeholder 0.01–0.07). Uncapped, the expected count is about 1.3 at tau 0.5 and about 6 at tau 0.3.
  - Decoys fire with probability 0.80 per pass (placeholder 0.28–0.55).
  - `loc_sigma` is about 0.6 m (placeholder 1.0–1.5 m).
  - `log_lr` of the score is -1.2 at 0.6 and +3.7 at 0.9.

**PRIORs (not calibrated):** `view_prior`, exposure 4 ms, attitude/yaw/height errors, the mission altitude of 3 m, the clutter factor (1 + rho) in `p_false_alert`, and the stand-visibility geometry (kept from the placeholder).

## 5. Limits (state these with any use)

1. The detector is COCO-pretrained and not fine-tuned. A fine-tuned or aerial-pretrained detector moves the size curve left; published SAR fine-tunes reach recall 0.75–0.89 on HERIDAL and SARD. The GPU was limited to inference here.
2. Only oblique views (about 45 deg) at NOMAD's sites and seasons. No nadir, no under-canopy horizontal views, no thermal.
3. Frames are downsampled from 5.4K, so they are cleaner than a native 1080p sensor (less noise and compression). Blur is synthetic and horizontal only.
4. The false-alert counts are upper bounds (only the actor is labelled). Lighting is confounded with site.
5. Latency was measured with eager PyTorch on a shared GPU. The Orin numbers are scaled estimates from cited papers.

## 6. Scope extension: mission-relevant perception (orchestrator request, 2026-11-09 late)

### 6.1 Controlled sweeps: what is covered and what is not

| axis | how swept | person-level effect (tiled, tau 0.3 unless noted) | status |
|---|---|---|---|
| flight speed x exposure | blur kernel L = f·v·t/R + f·ω·t, 0/3/6/12/24 px (§2) | recall 0.17 -> 0.035; FP/frame 0.68 -> 1.97 | done (synthetic blur, horizontal only) |
| viewing geometry | NOMAD distance 10–90 m at about 45 deg | recall 0.46 -> 0.025 | oblique only. **Nadir and near-horizontal are not measured**; renders top_30m/top_50m are queued in RERUN_PLAN |
| foliage occlusion | NOMAD visibility label | recall 0.075 (10–30 %) -> 0.48 (100 %) | done |
| sensor resolution | effective resolution: native tiles vs 1/3-scale full frame | recall 0.172 vs 0.041; size curve shifts by about 3x in px | partial. A 720p / 4K re-sample sweep was not run in the cap |
| compute contention | navigation-load proxy concurrent with the detector (§3) | p99 39 -> 52 ms (tiled fp16) | done, on a shared GPU |

**Effective-resolution rule for the sim.** Detection is a function of person pixel height in the *detector's input*. Full-frame 640 equals the tiled curve evaluated at h/3, which the table above confirms within the CIs: full at 64–128 px gives 0.225, and tiled at 16–32 px gives 0.166, at 32–64 px 0.309. Use `p_detect(h_px * input_scale)`.

**Known misfit.** Under leave-one-site-out the model's blur slope is too shallow: at 24 px it predicts 0.047 where 0.015 was observed (tau 0.5; `tau05/calibration.png`, middle panel). The `log1p(blur/h)` term under-weights blur on large persons. A comparison of alternative blur features (linear-in-px, `log1p(b/sqrt h)`) is scripted in `blur_features.py` but was not run within the cap. Until refit, treat blur predictions above 12 px as optimistic.

### 6.2 Single-frame vs temporal vs evidence accumulation vs adaptive (`compare_temporal.py`, `temporal/`)

Setup:
- The unit is a held-out-site run of 4 frames, at most 0.4 s apart: 171 runs, 12 actors, 5 sites.
- The detector is frozen (COCO weights, never tuned on NOMAD).
- Every evidence parameter (score densities, track prior, detection model) was fitted on the 4 fit sites only.
- Compute is counted in 640x640 tile inferences per run.

| method | tiles / run | blur 0: person recall | blur 0: false reports / run | blur 12: person recall | blur 12: false reports / run |
|---|---:|---:|---:|---:|---:|
| single-frame, tau 0.5, OR over 4 frames | 24 | **0.228** | 0.082 | 0.117 | 0.170 |
| tracking-by-detection (tau 0.3, >= 2 hits) | 24 | 0.263 | 0.433 | 0.129 | 1.78 |
| evidence accumulation (Bayesian log-odds, correlated repeat views) | 24 | 0.088 | 0.006 | 0.035 | 0.000 |
| full-frame 640 on all 4 frames | 4 | 0.058 | 0.000 | n/a | n/a |
| tiled on 1 of 4 frames (frame skip) | 6 | 0.164 | 0.047 | 0.076 | 0.123 |
| adaptive (full if expected h >= 64 px, else tiled) | 19.9 | 0.152 | 0.076 | n/a | n/a |

Findings:
- **Resolution beats frame rate at matched compute.**
  - Tiling one frame in four (6 tiles) gets 0.164 recall; full-frame on all four (4 tiles) gets 0.058.
  - Correlated repeat views make extra low-resolution looks nearly worthless, which matches rho of about 0.6.
  - So for the mission: spend compute on pixels on target (tiling, ROI crops, slower flight near candidates), not on frame rate.
- **Tracking by detection** (>= 2 hits at a low threshold) adds a little recall but multiplies false reports by 5–10x, because false alerts are persistent decoys: 80 % recur, so k-of-n confirmation does not reject them.
- **Evidence accumulation with fit-site parameters is miscalibrated on held-out sites at blur 0.**
  - Track-level ECE is 0.233 with log loss 0.71 (`temporal/evidence_reliability_blur0.png`); the held-out base rate is 0.41 versus a lower fit-site prior.
  - At blur 12 it is calibrated (ECE 0.054, log loss 0.28).
  - It is very conservative: it nearly eliminates false reports but loses recall.
  - This is the site-shift effect from §4 again. The accumulator needs the site offset (`ObsModel.draw_scene`, or online estimation of the site's base rate) before its posterior can be trusted for verification decisions.
- **Adaptive inference** sits in between at 83 % of the compute: when the expected size is large, full-frame inference is cheap and loses little. The 64 px switch point is a heuristic, not optimized.

### 6.3 Calibrated spatial outputs: map-update API (`obs_model.SearchMaps`, `obs_model.new_maps`)

This feeds Tools/RoleSpec (recon / verify) and Tools/MissionSim. RoleSpec does not exist yet, so the API is standalone and pure numpy; tests are in `test_obs_model.py` (12 pass).

**Calls:**
- `update(cam_xy, alt_m, yaw, pitch_deg, detections, speed, yaw_rate, visible, lighting, low_sun)` folds in one look.
- `project_detection(...)` produces a ground detection with its location covariance (from `loc_error`).
- `posterior()` is the detection map.
- `coverage()` is the coverage-confidence map.
- `view_classes()`, `verify_needed(p_lo, p_hi, cov_min)`.
- `candidates(p_min)` returns survivor candidates as {x, y, **confidence**, peak, **cov** (2x2 m^2, separate from confidence), coverage, views}.

**Per-cell model:**
- A pinhole footprint on flat ground.
- Per-cell p_detect from apparent size, blur, view angle and visibility (canopy occlusion can be passed per cell).
- Correlated repeat looks within a view class (beta-binomial rho; 4 view classes by bearing), with independent classes across bearings.
- A Bernoulli-target likelihood ratio LR = (1 - p_inc) + p_inc · sum_j f1(s_j) N(z_j; c, S_j) / (lambda_fa f0(s_j)).
- Frame-alignment error (`sigma_align_m`, default 0.5 m) is added to every detection covariance.
- Positive evidence from same-view repeats is tempered by the n_eff increment, because decoys are persistent.

**Calibration status.**
- Map *coverage* inherits the per-frame model's leave-one-site-out calibration (§4).
- The map *posterior* has been checked only in image space as track-level evidence (§6.2): calibrated at blur 12, not at blur 0 under site shift.
- A ground-frame reliability check needs georeferenced held-out data. That means CRANE renders with GT (RERUN_PLAN), or WiSARD/NOMAD with drone telemetry, which NOMAD lacks. **Not done.**

### 6.4 Jetson Orin Nano: ESTIMATE with uncertainty, plus a ready-to-run script

**No hardware here. Clock or power limiting on this GPU is not permitted** for this user (`nvidia-smi -lgc`: no permission), so no throttled desktop proxy was measured.

Scaling model (Orin Nano Super, MAXN_SUPER, TensorRT FP16):
- Per 640 tile engine time t1 is 12.3–13.8 ms (cited: 81 FPS, arXiv 2606.14005; p95 13.8 ms, arXiv 2608.00927).
- Batch-6 efficiency eta is 0.85–1.0.
- Host pre/post on the A78AE cores t_pp is 5–10 ms.

| config | p50 ESTIMATE (ms) | p95 (x1.8, measured desktop tail ratio under nav load) | p99 (x2.3) |
|---|---:|---:|---:|
| full 640 | 17–24 | 31–43 | 39–55 |
| tiled 6x640 | 68–93 | 122–167 | 156–214 |
| tiled 6x640, non-Super clocks (x1.6 on the GPU part) | 105–143 | 189–257 | 242–329 |

**Uncertainties:**
- The cited numbers come from two papers with different JetPack versions and power modes. Neither is reproduced here.
- The tail ratios come from a shared desktop GPU and may be worse on unified memory.
- INT8 would cut engine time by about 1.2–1.5x (the YOLO26n INT8/FP16 ratio is 0.83) and was not modelled.

**Ready to run:** `Tools/Vision/orin_bench.py`. It exports static batch-1 and batch-6 ONNX on the desktop (`--export`), builds TensorRT engines with trtexec on the Jetson, and runs the same end-to-end p50/p95/p99 idle and under CPU navigation load, logging `nvpmodel`.

### 6.5 Not done within the cap (next steps, in priority order)

1. Ground-frame map calibration on georeferenced held-out data. This needs the RERUN_PLAN GT fields, or WiSARD (MIT) with flight logs.
2. Nadir and near-horizontal viewpoint: renders top_30m/top_50m, Okutama 90 deg, ForestPersons (under-canopy, CC BY-NC-SA).
3. Blur-feature refit (see 6.1). A 720p / 4K resolution re-sample sweep.
4. Site-adaptive evidence accumulation (online base-rate estimate) before verify decisions use the posterior.
5. A light aerial fine-tune of RT-DETR-R18 (Apache-2.0) on WiSARD or NOMAD fit-site actors, with the detector frozen before held-out evaluation. GPU training was outside this task's inference-only rule.

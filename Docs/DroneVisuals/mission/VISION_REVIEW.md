# Aerial SAR Person Detection: Literature Review for the CRANE Observation Model

Status: desk review, 2026-11-09. Sources were web pages and papers only. No datasets or weights were downloaded.
Conventions: "unverified" means no primary source was reached. "unstated" means the primary source was read and gives no value.
"(inferred)" marks a value derived by this review and not printed in the source.

## 1. Datasets

| Dataset | Images / boxes | Resolution | Altitude, viewpoint | Modality | Occl./trunc. labels | Terrain / season / light | License (where stated) | Download, login | Leakage-safe grouping |
|---|---|---|---|---|---|---|---|---|---|
| WiSARD (Broyles, Hayner, Leung, IROS 2022) [W1] | Paper: 33,786 RGB + 22,156 thermal (~55.9k), with 15,453 synced pairs. Website: 26,862 visual + 29,989 thermal [W2] (counts disagree). Box count unstated in paper; ForestPersons table gives 44,588 img / 74,204 boxes [F2] | RGB 3840x2160 (some 1920x1080 / 2720x1530); thermal 640x512 [W1] | 20-120 m AGL. Aspect angle "varied", gimbal angle unstated [W1]. ForestPersons calls it oblique [F2] | RGB + LWIR | None. Per-folder canopy tag (above/below/none) [W1] | 12 WA-state sites: forest, field, rocky, coastal, snow; all 4 seasons; day 35.7k, night 7.1k, dawn 1.8k, dusk 9.9k [W1] | MIT text on dataset site, "Copyright 2022 Broyles, Hayner, Leung" [W2]. Paper does not state a data license | Public Google Drive links, no login stated, ~40.5 GB [W2] | Data are grouped in folders with one .yaml per folder (collection/flight level). Folder naming unspecified [W1]. Paper split was 60/20/20 with method unstated, so it must be re-split by folder |
| HERIDAL (Bozic-Stulic, Marusic, Gotovac, IJCV 2019) [H1] | ~500 labelled full images per site; 68,750 patches (29,050 pos / 39,700 neg) [H1]. Later works use 1,546-1,600 full images / 3,194 boxes [F2][A1]. Official test set: 101 img / 337 persons [A1] | 4000x3000 [H1] | 40-65 m (secondary [S1]); top-down per [F2]. Camera model unverified | RGB | None [F2] | Mediterranean/sub-Mediterranean mountain and karst in Croatia / BiH, summer-dominant (unverified) | CC BY 3.0 Unported, stated on official page [H1] and in Accenture AIR README [H2] | heridal.zip on http://ipsar.fesb.unist.hr/HERIDAL%20database.html (HTTPS refused during this review, so not verified live). No login stated | No flight/sequence IDs known. Fixed official 101-image test split [A1] |
| SARD (Sambolek, Ivasic-Kos, IEEE Access 2021) [S2][S3] | 1,981 frames / 6,532 boxes [F2]. Roboflow re-export has 5,755 images, likely augmented [A1] | Frames from 1920x1080 video (secondary) | 5-50 m (secondary [S1]); oblique [F2]. DJI Phantom 4A per Kaggle re-upload text [S4] | RGB | None. Pose classes: standing, walking, running, sitting, lying, not defined [S2] | Macadam roads, quarry, low/high grass, forest shade. "Corr" extension adds synthetic fog, snow, blur [S2] | CC BY 4.0 per IEEE DataPort page [S2] | IEEE DataPort needs a subscription and login [S2]. Kaggle/Roboflow mirrors need an account (Kaggle label: unstated; SARD-2 says MIT) [S4] | Frames come from 8 videos, ~35 min [S2]. Group by source video. The Roboflow export mixes augmentations, so do not use its random split |
| ForestPersons (Kim et al., ICLR 2026) [F1][F2] | 96,482 img / 204,078 boxes (train 67,686, val 18,243, test 10,553) [F1] | unstated (GoPro HERO9, Sony A57, See3CAM) [F1] | 1.5-2.0 m camera height (under-canopy MAV proxy). Not an aerial altitude dataset | RGB. IR is a separate, unpaired set (etri/ForestPersonsIR) [F1] | Visibility 20/40/70/100 per box, plus pose (stand/sit/lie) [F1] | Forest, multiple seasons and weather [F2] | CC BY-NC-SA 4.0. SAR use explicitly allowed. Trained models also non-commercial [F1] | HuggingFace etri/ForestPersons. Gated: login and agreement required [F1] | Splits are at video-sequence level (377 sequences), difficulty-balanced [F2] |
| VisDrone2019-DET (Zhu et al., TPAMI 2021) [V1] | 10,209 static img (6,471 / 548 / 1,610 test-dev / 1,580 challenge) [V2]; 288 clips, 261,908 frames; >2.6M boxes total [V1] | ~2000x1500 typical (unverified) | Low-altitude urban/suburban, mostly oblique [F2]. Altitude not labelled | RGB | Yes. occlusion 0 = none, 1 = 1-50%, 2 = 50-100%; truncation 0 = none, 1 = 1-50%. Classes: pedestrian(1) = standing/walking, people(2) = other poses [V3] | 14 Chinese cities, urban/rural, day/night, varied weather [V1] | CC BY-NC-SA 3.0, copyright AISKYEYE / Tianjin Univ. (per Voxel51 card [V4] and Ultralytics [V2]). Official README states no license [V1] | Google Drive / BaiduYun links in the official README. Login not stated [V1] | DET filenames carry a sequence prefix (e.g. `0000001_02999_d_0000005`). Group by prefix (format unverified against the README) |
| Okutama-Action (Barekatain et al., CVPRW 2017) [O1] | 43 sequences, 77,365 4K frames [O1]; 524,649 boxes [F2] | 3840x2160 video; 1280x720 frame release [O2] | 10-45 m, camera angle 45 deg or 90 deg. Per-sequence metadata gives angle, speed, altitude [O1] | RGB | None. 12 action labels, multi-label [O1] | Single baseball field in Okutama, Japan; daytime [O1] | unstated in repo [O2]. Aggregator says non-commercial [O3] (unverified) | Dropbox shared folder linked from GitHub (miquelmarti/Okutama-Action). No login mentioned [O2] | 43 sequence IDs with 33 train / 10 test. Test ground truth reportedly withheld (2018 report, unverified) |
| NOMAD (Russell Bernal, Scheirer, Cleland-Huang, WACV 2024) [N1][N2] | 42,825 annotated keyframes from 100 actors x 5 distances. One box + visibility label per frame [N1] | 5472x3078 (5.4K, 30 fps, H.265), 8 mm focal [N1] | Nominal 10/30/50/70/90 m, set "horizontally and vertically from the expected starting point of the actor". So AGL ~ d and ground offset ~ d, slant ~1.41 d, ~45 deg depression (inferred, consistent with Table S3 GSD 4.24-38.18 mm/px). Gimbal pitch not fixed and not reported. Drone/camera model and sensor size unstated (inferred pixel pitch ~2.4 um, 1-inch class) [N1] | RGB | 10 visibility bins (upper bounds 10..100). Score = sum of per-body-part percentages (Fig. 4), half credit if under half of a part is visible. Shadows and own clothing are not occluders [N1] | 12 sites: forest park, farms, quarry, lake shore, golf course, schools, paintball. Summer/autumn/snowy winter; morning/afternoon/evening; EV recorded [N1] | CC BY 4.0, stated in repo README [N2] | Public Google Drive folder, no form stated [N2] | Actor IDs (Actor001-100) and distance IDs (`_a10`...) in paths. 10-fold actor split in paper [N1][N2] |
| AIResQ (Buttner et al., Sci. Data 2026) [R1] | 9,788 thermal img / 17,550 boxes, plus a separate real-SAR benchmark of 1,988 img / 2,643 boxes at 640x512 [R1] | 2048x1536 / 1024x768 [R1] | 50-120 m from cable cars, masts, towers (handheld, not a drone) [R1] | LWIR | None. Occluded/truncated people are boxed but not flagged [R1] | Alpine, full year [R1] | Article CC BY 4.0. Data license unstated. Zenodo "available upon request" [R1] | Zenodo (records disagree; access request implied) [R1] | No flight IDs |
| POP (Song et al., Sci. Data 2025) [P1] | 8,767 img / 26,811 boxes [R1] | 1280x1024 [R1] | 30-70 m (secondary, unverified) | LWIR | Partial-occlusion focus. Level scheme unverified | Wilderness, tree occlusion [P1] | unverified (Nature page blocked by login redirect) | unverified | unverified |
| Others (brief) | VTSaR / A-VTSaR (J. Remote Sens. 2025): 8,032 vis + 5,164 IR, unaligned, from helicopter gimbal; Baidu Pan; license unstated [T1]. SeaDronesSee: maritime, CC0 1.0 per repo [T2]. TinyPerson: web video frames; repo MIT, data rights unclear, non-commercial research recommended [T3]. CRASAR Wu-Murad real-search imagery: github.com/CRASAR/WiSAR [M1] | | | | | | | | |

## 2. Detectors and reported results

| Detector (license) | Data / split | Reported metric | Source |
|---|---|---|---|
| YOLOv5 (AGPL-3.0) | WiSARD, 60/20/20, split method unstated, RGB tiled 512 | RGB mAP50 0.892, recall 0.845; thermal mAP50 0.977, recall 0.959 | [W1] |
| Faster R-CNN R50-FPN v2 / RT-DETR-L / YOLO11m, all COCO-init | HERIDAL official 101-img test (tiled 1024) | Recall 0.745 / 0.792 / 0.769; mAP50 0.785 / 0.835 / 0.803 | AegisBench [A1] |
| same | SARD Roboflow export, group-aware split | Recall 0.879 / 0.888 / 0.891; mAP50 0.896 / 0.929 / 0.928 | [A1] |
| same, synthetic corruptions | HERIDAL / SARD | Motion blur s1 recall on HERIDAL 0.003-0.107 vs SARD 0.772-0.792. Low light s3 recall 0 on both. Smoke s3 HERIDAL 0.148-0.407 | [A1] |
| Faster R-CNN, cross-dataset | trained on SARD / HERIDAL / WiSARD, tested on ForestPersons | AP 3.0 / 0.2 / 11.3 (vs in-domain 58.6 / 35.0 / 18.5) | [F2] |
| Faster R-CNN, real MAV under-canopy test | trained on ForestPersons / WiSARD / SARD / HERIDAL | AR 69.0 / 50.0 / 34.8 / 0.3 | [F2] |
| YOLO11 / DINO / Deformable R-CNN / YOLOX | ForestPersons test (sequence split) | AP 65.6 / 65.3 / 66.3 / 51.0; AR 71.7 / 77.7 / 71.3 / 58.2 | [F2] |
| YOLOv8l, FRCNN-R101, RetinaNet-R101 (COCO-only, no fine-tune) | NOMAD, 10-fold by actor | mAP falls sharply with distance 10 to 90 m, and with lower visibility even at 10 m. Values are in a figure only | [N1] |
| YOLOv8 / YOLO11 / RT-DETR / RetinaNet / FRCNN | trained on AIResQ, tested on real-SAR thermal benchmark (~10 px tall people) | Best mAP50 0.55 (YOLOv8/11). Best non-AIResQ training: RT-DETR on POP, 0.31. Humans beat the best model | [R1] |
| YOLOv5s variants (AGPL-3.0) | HERIDAL, VisDrone pretrain | mAP50 0.665 (HERIDAL only), rising to 0.738 with VisDrone pretrain | [D1] |
| EfficientDet | HERIDAL vs real Wu-Murad search | Near-SOTA on HERIDAL, but too many false positives in the real search | [M1] |

Licenses for onboard-capable families:
- Ultralytics YOLOv5/v8/YOLO11/YOLO26: AGPL-3.0 or Enterprise [L1].
- YOLOv10 (THU-MIG): AGPL-3.0, inherited from the Ultralytics codebase [L2].
- Apache-2.0 alternatives:
  - RT-DETR and RT-DETRv2 (lyuwenyu/RT-DETR)
  - D-FINE (Peterande/D-FINE)
  - DEIM (Intellindust-AI-Lab/DEIM)
  - YOLOX (Megvii) [L2][L3]
- HF `PekingU/rtdetr_r18vd` is tagged apache-2.0: COCO AP 46.5 (49.2 with O365 pretrain), 217 FPS at bs=1 [L4]. The upstream repo gives this as T4 TensorRT FP16.
- These license facts come from third-party summaries plus the HF card. Check the GitHub LICENSE files before release.

What drives misses:
1. **Pixel size and altitude.** At 90 m NOMAD's face area is ~24 px, against 1,981 px at 10 m [N1]. The AIResQ benchmark people are ~10 px tall and cap mAP50 at 0.55 [R1]. NOMAD mAP falls monotonically with distance [N1].
2. **Occlusion and canopy.** NOMAD mAP drops "drastically" with occlusion even at 10 m [N1]. Overhead-trained models fail under canopy (AP <= 11.3) [F2]. WiSARD has only 2,924 below-canopy images [W1].
3. **Pose.** Sitting and lying AP fall when training is standing-only [F2].
4. **Motion blur, as kernel length relative to person size.** HERIDAL people collapse at s1 while SARD people degrade gracefully [A1].
5. **Light.** Low-light s3 gives zero recall for all three detectors, and only 2 of 759 real night people were found [A1].
6. **Thermal vs RGB.** On WiSARD, thermal recall is 0.959 vs 0.845 for RGB [W1]. But tiny thermal targets stay hard [R1].
7. **Season.** Winter-only training generalises poorly [F2].

## 3. Jetson Orin Nano latency references

| Model, precision, runtime | Orin Nano | Reference GPU | Notes | Source |
|---|---|---|---|---|
| YOLO11n, TRT FP16, 640, inference only | 4.53 ms (Orin Nano Super, docs figure quoted). Maintainer reproduced 4.5 ms on coco128, MAXN + jetson_clocks, ultralytics 8.3.222. User's JP6.2 board saw 15-16 ms until the engine was re-exported on device | T4 TensorRT10: 1.5 ms [L1] | Super / T4 ratio ~3.0x | [J1][L1] |
| YOLO26n, TRT FP16 / FP32 / INT8, 640 | Super: 4.57 / 7.53 / 3.80 ms (JP6.1, ultralytics 8.4.33). AGX Orin 64GB FP16: 2.62 ms | - | Pre/post-processing excluded. Power mode not stated | [J2] |
| RT-DETR-R18, TRT FP16, 640, bs1 | Super, 25 W native mode: 72.7 FPS, p95 13.79 ms. AGX Orin 50 W: 105.1 FPS, p95 9.73 ms | T4 TRT FP16: 217 FPS (4.6 ms) [L4] | Super / T4 ratio ~2.7-3.0x. Engine-only timing | [J3] |
| RT-DETR-R18, trtexec FP16, bs1 | 80.99 FPS (~12.3 ms). Orin Nano variant and power mode unstated | - | - | [J4] |
| YOLO11n / YOLOv8n, TRT 8.6 FP16, bs1 | Orin Nano 8GB "40 TOPS" (implies non-Super): 26.8 ms / 33.7 ms | RTX 4090: 163 / 128 FPS (NMS included) | Ratio ~4.4x. The 4090 FPS is far below engine-only speed, so treat as end-to-end and low-confidence | [J5] |
| YOLOv8n, TRT FP16, 640, GPU-only | Super (MAXN_SUPER): 4.43 ms, 14.1 W; INT8 3.49 ms | - | Personal blog | [J6] |
| YOLO11n, TRT FP16, 640 | Orin Nano 8GB: GPU median 7.99 ms; full serial app 37.35 FPS | - | Shows app overhead of ~3x engine time | [J7] |
| YOLO11n, TRT FP16, DeepStream | - | RTX 4090 Laptop: ~1.9 ms | Forum post, weak | [J8] |
| PyTorch FP32 (no TRT) | Super: YOLO11n 43.8 ms, RT-DETR-R18 98.3 ms, D-FINE-n 85.5 ms, DEIM-n 88.5 ms | - | Contributor-reported. Resolution and JetPack not on page | [J9] |

Hardware specs:

| Spec | Orin Nano 8GB, original (JP <= 6.1) | Orin Nano 8GB "Super" (JetPack 6.2 MAXN_SUPER) | RTX 4070 Ti SUPER |
|---|---|---|---|
| INT8 TOPS | 40 sparse / 20 dense [K1] | 67 sparse / 33 dense [K1] | 706 "AI TOPS" = FP8 with sparsity, so ~353 dense FP8 (inferred) [K3][K4] |
| GPU clock | 635 MHz [K2] | 1020 MHz (CNX says 1,050) [K2] | 2.61 GHz boost [K3] |
| Memory bandwidth | 68 GB/s [K2] | 102 GB/s [K2] | 672 GB/s [K3] |
| Other | - | Power modes 7 / 15 / 25 W [K2] | 44.1 FP32 TFLOPS, 285 W [K3] |

Scaling guidance (inferred):
- The peak-spec ratio from 4070 Ti SUPER to Orin Nano Super is ~10.7x on dense 8-bit and ~6.6x on bandwidth. Measured small-detector ratios are much smaller, because launch and memory overheads dominate. Published same-model ratios are:
  - T4 to Super: ~3x (YOLO11n, RT-DETR-R18) [J1][J3][L1][L4]
  - RTX 4090 to non-Super: ~4.4x end-to-end [J5]
- Recommendation: measure the actual RTX 4070 Ti SUPER with `trtexec --fp16` on the same ONNX file, then apply a 3-5x factor to reach Orin Nano Super. Use ~1.6x more for the non-Super/JP <= 6.1 clock profile, which is the 1020/635 MHz clock ratio. Treat this as an upper bound, since bandwidth only scales 1.5x.
- Budget roughly 2-3x engine time for the full camera pipeline [J7].
- Re-build engines on the device itself [J1].

## 4. Implications for the CRANE observation model

- **Strata to model.**
  - Person pixel height or GSD, from altitude, slant range, focal length and resolution. This is the primary axis [N1][R1].
  - Visibility/occlusion fraction: canopy above / below / none [W1]; NOMAD 10-bin visibility [N1]; VisDrone 0/1/2 [V3]; ForestPersons 20/40/70/100 [F1].
  - Pose: standing vs sitting/lying [F2][S2].
  - Look angle: 45 deg vs 90 deg [O1].
  - Motion blur in pixels relative to person size [A1].
  - Illumination: day / dusk / night [W1][A1].
  - Modality: RGB vs thermal [W1][R1].
- **Recommended for leakage-safe splits.**
  - NOMAD: group by actor ID. It gives distance x visibility strata directly, CC BY 4.0, no login.
  - WiSARD: group by collection folder/yaml. It has altitude 20-120 m, RGB + thermal and canopy tags; MIT, no login.
  - Okutama-Action: group by sequence. It has per-sequence altitude and angle metadata.
  - VisDrone-DET: group by sequence prefix. Gives occlusion strata; urban, so a weak proxy for wilderness.
  - HERIDAL: official fixed test set only. It has no grouping IDs and its adjacent images may be correlated (unverified).
  - SARD: group by source video. Avoid the augmented Roboflow random split [A1].
  - Never random-split adjacent frames, given WiSARD's every-6th-frame-at-30-Hz annotation [W1].
- **Licenses.** Record the license with every experiment.
  - Non-commercial / share-alike: VisDrone (CC BY-NC-SA 3.0), ForestPersons (CC BY-NC-SA 4.0, extends to trained models), Okutama (reported non-commercial, unverified). These are acceptable for research experiments only.
  - Attribution only: HERIDAL (CC BY 3.0), NOMAD (CC BY 4.0), SARD (CC BY 4.0 on DataPort).
  - MIT: WiSARD.
  - Model code: RT-DETR via HF transformers is Apache-2.0. Prefer RT-DETR/D-FINE/DEIM/YOLOX over Ultralytics (AGPL-3.0) for anything CRANE ships under GPL-3.0. AGPL weights in a networked service carry source-disclosure duties [L2].
- **Calibrate with recall, not mAP alone.** Recall gaps are large: HERIDAL ~0.75-0.79 vs SARD ~0.88-0.89 with the same detectors [A1]. Benchmark recall overstates field performance [M1][F2].

## References

- [W1] Broyles et al., WiSARD, IROS 2022. https://arxiv.org/html/2309.04453v1
- [W2] WiSARD site. https://sites.google.com/uw.edu/wisard/
- [H1] HERIDAL official page. http://ipsar.fesb.unist.hr/HERIDAL%20database.html (content via search snippet; live fetch refused)
- [H2] Accenture AIR README. https://github.com/Accenture/AIR/blob/master/README.md
- [S1] Person Recognition in Aerial Surveillance survey (altitude table). https://arxiv.org/html/2511.17674v1
- [S2] SARD, IEEE DataPort. https://ieee-dataport.org/documents/search-and-rescue-image-dataset-person-detection-sard
- [S3] Sambolek & Ivasic-Kos, IEEE Access 9:37905, 2021. doi:10.1109/ACCESS.2021.3063681
- [S4] Kaggle SARD re-upload. https://www.kaggle.com/datasets/nikolasgegenava/sard-search-and-rescue
- [F1] ForestPersons dataset card. https://huggingface.co/datasets/etri/ForestPersons
- [F2] ForestPersons paper. https://arxiv.org/html/2603.02541v1
- [V1] VisDrone README. https://github.com/VisDrone/VisDrone-Dataset
- [V2] Ultralytics VisDrone page. https://docs.ultralytics.com/datasets/detect/visdrone
- [V3] VisDrone DET toolkit. https://github.com/VisDrone/VisDrone2018-DET-toolkit
- [V4] Voxel51 card. https://huggingface.co/datasets/Voxel51/VisDrone2019-DET
- [O1] Okutama-Action paper. https://arxiv.org/pdf/1706.03038
- [O2] Okutama-Action repo. https://github.com/miquelmarti/Okutama-Action
- [O3] hyper.ai listing. https://hyper.ai/en/datasets/5714
- [N1] NOMAD paper. https://arxiv.org/html/2309.09518v2 (WACV 2024)
- [N2] NOMAD repo. https://github.com/ArtRuss/NOMAD
- [R1] AIResQ, Sci. Data 2026. https://pmc.ncbi.nlm.nih.gov/articles/PMC13338175/
- [P1] POP, Sci. Data 12:300, 2025. https://www.nature.com/articles/s41597-025-04600-0
- [T1] VTSaR repo. https://github.com/zxq309/VTSaR ; survey https://spj.science.org/doi/10.34133/remotesensing.0474
- [T2] SeaDronesSee. https://github.com/Ben93kie/SeaDronesSee
- [T3] TinyPerson. https://github.com/Muqianli/TinyBenchmark ; https://huggingface.co/datasets/shijli/tinyperson
- [M1] Manzini & Murphy, ICCVW 2023. https://arxiv.org/abs/2307.14527
- [A1] AegisBench. https://arxiv.org/html/2611.00077
- [D1] Drones 2025, YOLO SAR small objects. https://cris.unibo.it/retrieve/0a6fbcaf-3fed-43e2-a1e7-ea9937dd6b7d/drones-09-00514-2.pdf
- [L1] Ultralytics YOLO11. https://docs.ultralytics.com/models/yolo11/
- [L2] LibreYOLO license guide. https://www.libreyolo.com/articles/yolo-licenses-explained
- [L3] Roboflow D-FINE page. https://playground.roboflow.com/models/ustc/d-fine ; https://github.com/lyuwenyu/RT-DETR
- [L4] HF RT-DETR-R18 card. https://huggingface.co/PekingU/rtdetr_r18vd
- [J1] Ultralytics issue 22479. https://github.com/ultralytics/ultralytics/issues/22479
- [J2] Ultralytics Jetson guide. https://docs.ultralytics.com/guides/nvidia-jetson/
- [J3] AgriJetsonBench. https://arxiv.org/html/2608.00927v1
- [J4] FINE fusion paper. https://arxiv.org/pdf/2606.14005
- [J5] LAF-YOLOv10, Tables 1 and 8. https://arxiv.org/html/2602.13378v1
- [J6] H. Choi blog. https://hokwangchoi.com/blog/vision-benchmarks/
- [J7] Turing Pi. https://turingpi.com/yolo-jetson-orin-nano-tensorrt-benchmarks/
- [J8] NVIDIA forum. https://forums.developer.nvidia.com/t/sm-utilization-6-8x-higher-on-rtx-4090-vs-l40s-for-identical-yolo11n-deepstream-9-0-pipeline-expected-behavior-or-config-issue/374874
- [J9] VisionAnalysis Orin Nano Super. https://www.visionanalysis.org/hardware/jetson_orin
- [K1] NVIDIA Orin Nano datasheet DS-11105-001 v1.5 (mirror). https://plink-ai.com/uploads/20260529/0cf0a3910f554335f1b5138246b1775f.pdf
- [K2] CNX Software on Orin Nano Super. https://www.cnx-software.com/2024/12/18/249-nvidia-jetson-orin-nano-super-developer-kit-targets-generative-ai-applications-at-the-edge/ ; https://things-embedded.com/us/white-paper/nvidia-jetson-orin-nano-super-67-tops-update/
- [K3] Tom's Hardware, RTX 4070 Ti Super review. https://www.tomshardware.com/pc-components/gpus/nvidia-geforce-rtx-4070-ti-super-review
- [K4] bestgpusforai comparison. https://www.bestgpusforai.com/gpu-comparison/4070-ti-super-vs-4080

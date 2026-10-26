# GOAL

Build CRANE into a physically grounded, reproducible sim-to-real research platform (Unity 6/HDRP + ROS 2 via `astro_dock`) usable by roboticists and programmatically by an LLM.

**Research question (falsifiable):** Do calibrated transient actuator/delivery timing models plus uncertainty improve (a) held-out navigation-configuration selection, (b) robust transfer, and (c) abstention on unsafe recommendations, under limited data and trial budgets, compared with strong black-box/contextual baselines that use uncalibrated or steady-state-only models?

Non-claims: "LLM tunes Nav2", "RL tunes Nav2", "cross-domain Unity sim", or "automated domain randomization" are not novelty on their own.

Phases: A audit/contracts → B CRANE-owned ROS packages → C replay/fit/uncertainty → D optimizer API + baselines → E runtime qualification → F sequential RL (only if shown superior).

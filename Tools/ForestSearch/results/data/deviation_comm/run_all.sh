#!/bin/bash
cd /home/lunarz/worktrees/crane-forest-sar/Tools
set -x
date
python3 -u -m ForestSearch.deviation_comm b --episodes 100 --names ORACLE IMPLICIT HEALTH HYBRID_P DEV DEV_TASK DEV_INFO DEV_SD DEV_SD_B12 --only0 DEV_TASK DEV_INFO --out /home/lunarz/crane_ml/Docs/ResearchOrchestration/AGENT_HANDOFFS/deviation_comm_raw/dev_b.json
date
python3 -u -m ForestSearch.deviation_comm a --episodes 100 --names ORACLE IMPLICIT DEV --out /home/lunarz/crane_ml/Docs/ResearchOrchestration/AGENT_HANDOFFS/deviation_comm_raw/dev_a.json
date
python3 -u -m ForestSearch.deviation_comm c --episodes 100 --links d0_loss0 --names IMPLICIT HEALTH HB_TO DEV DEV_SD DEV_SD_B12 --out /home/lunarz/crane_ml/Docs/ResearchOrchestration/AGENT_HANDOFFS/deviation_comm_raw/dev_c.json
date

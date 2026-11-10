#!/bin/bash
cd /home/lunarz/worktrees/crane-forest-sar/Tools
date
python3 -u -m ForestSearch.deviation_comm a --episodes 300 --links d1_loss0 d0_loss0 --names ORACLE IMPLICIT DEV DEV_TASK --only0 DEV_TASK --out /home/lunarz/crane_ml/Docs/ResearchOrchestration/AGENT_HANDOFFS/deviation_comm_raw/dev_a300.json
date

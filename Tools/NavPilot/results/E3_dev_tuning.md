# E3 DEV tuning summary (layouts 300-301, 6 and 9 m/s; selected config per method)

_DEV only, used for selection; not reportable evidence. 4 episodes per family cell (2 layouts x 2 speeds), 16 per method._

| method (tuned config) | stale 6/9: coll, prog | drift 6/9 | occl 6/9 | stall 6/9 | all: coll, stuck, prog |
|---|---|---|---|---|---|
| opt_backup_t (k4) | 3/4, 5.2 | 1/4, 3.5 | 0/4, 1.6 | 0/4, 3.8 | 4/16, 6, 3.53 |
| reactive_plus (k4) | 1/4, 5.0 | 1/4, 5.8 | 2/4, 5.5 | 0/4, 3.9 | 4/16, 0, 5.04 |
| mpc_lc (k4) | 2/4, 5.5 | 0/4, 2.9 | 1/4, 1.7 | 0/4, 4.0 | 3/16, 6, 3.52 |
| ua_map (k0) | 3/4, 5.5 | 1/4, 6.0 | 0/4, 5.0 | 0/4, 3.7 | 4/16, 0, 5.07 |
| pas_opt (k0) | 0/4, 4.0 | 0/4, 2.9 | 0/4, 1.5 | 0/4, 2.7 | 0/16, 6, 2.77 |
| pas_full (k3) | 0/4, 4.8 | 0/4, 5.1 | 0/4, 3.9 | 0/4, 3.4 | 0/16, 0, 4.30 |

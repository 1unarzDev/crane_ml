# Protocol-choice game (common payoff)

instance=grid4x4-s7, link=burst30_L10_d1, episodes=600

## cost c = 0.0 per kB

| row \ col | INDEP | IMPLICIT | RESERVE | FULL | EVENT |
|---|---|---|---|---|---|
| INDEP | 0.618 | 0.685 | 0.618 | 0.618 | 0.685 |
| IMPLICIT | 0.635 | 0.720 | 0.648 | 0.657 | 0.702 |
| RESERVE | 0.618 | 0.657 | 0.673 | 0.668 | 0.677 |
| FULL | 0.618 | 0.658 | 0.672 | 0.670 | 0.673 |
| EVENT | 0.635 | 0.713 | 0.677 | 0.680 | 0.688 |

team optimum: IMPLICIT,IMPLICIT U=0.720
pure NE: [('IMPLICIT', 'IMPLICIT')]
worst NE / optimum: 0.720 / 0.720
(Monte Carlo noise ~ +/-0.01; ties within noise are not meaningful.)

## cost c = 0.05 per kB

| row \ col | INDEP | IMPLICIT | RESERVE | FULL | EVENT |
|---|---|---|---|---|---|
| INDEP | 0.618 | 0.685 | 0.607 | 0.593 | 0.669 |
| IMPLICIT | 0.635 | 0.720 | 0.638 | 0.633 | 0.687 |
| RESERVE | 0.608 | 0.647 | 0.654 | 0.636 | 0.647 |
| FULL | 0.595 | 0.636 | 0.642 | 0.627 | 0.632 |
| EVENT | 0.626 | 0.706 | 0.649 | 0.639 | 0.661 |

team optimum: IMPLICIT,IMPLICIT U=0.720
pure NE: [('IMPLICIT', 'IMPLICIT'), ('RESERVE', 'RESERVE')]
worst NE / optimum: 0.654 / 0.720
(Monte Carlo noise ~ +/-0.01; ties within noise are not meaningful.)

## cost c = 0.2 per kB

| row \ col | INDEP | IMPLICIT | RESERVE | FULL | EVENT |
|---|---|---|---|---|---|
| INDEP | 0.618 | 0.685 | 0.575 | 0.519 | 0.621 |
| IMPLICIT | 0.635 | 0.720 | 0.606 | 0.561 | 0.642 |
| RESERVE | 0.579 | 0.619 | 0.598 | 0.540 | 0.557 |
| FULL | 0.527 | 0.571 | 0.552 | 0.498 | 0.510 |
| EVENT | 0.601 | 0.683 | 0.565 | 0.518 | 0.578 |

team optimum: IMPLICIT,IMPLICIT U=0.720
pure NE: [('IMPLICIT', 'IMPLICIT')]
worst NE / optimum: 0.720 / 0.720
(Monte Carlo noise ~ +/-0.01; ties within noise are not meaningful.)

| family | split | plan | n | success | median t (s, successes) | failure causes |
|---|---|---|---|---|---|---|
| aisle_traffic | heldout | none | 3 | 0/3 | - | collision x3 |
| aisle_traffic | heldout | reserved | 3 | 1/3 | 50.4 | collision x1, stuck x1 |
| aisle_traffic | train | none | 5 | 4/5 | 31.8 | collision x1 |
| aisle_traffic | train | reserved | 5 | 3/5 | 44.7 | stuck x1, collision x1 |
| blocked_faces | heldout | fixed_face | 3 | 0/3 | - | face_blocked x3 |
| blocked_faces | heldout | free_face | 3 | 3/3 | 68.7 | - |
| blocked_faces | heldout | move_target | 3 | 3/3 | 78.1 | - |
| blocked_faces | train | fixed_face | 5 | 0/5 | - | face_blocked x5 |
| blocked_faces | train | free_face | 5 | 5/5 | 68.9 | - |
| blocked_faces | train | move_target | 5 | 5/5 | 76.2 | - |
| charging_contention | heldout | assigned | 3 | 3/3 | 121.3 | - |
| charging_contention | heldout | nearest | 3 | 0/3 | - | deadline x3 |
| charging_contention | train | assigned | 5 | 5/5 | 73.3 | - |
| charging_contention | train | nearest | 5 | 0/5 | - | deadline x5 |
| clutter_observer | heldout | observer | 3 | 3/3 | 53.8 | - |
| clutter_observer | heldout | self_search | 3 | 2/3 | 59.5 | not_found x1 |
| clutter_observer | train | observer | 5 | 5/5 | 55.4 | - |
| clutter_observer | train | self_search | 5 | 5/5 | 54.0 | - |
| friction_patches | heldout | around | 3 | 1/3 | 44.2 | deadline x2 |
| friction_patches | heldout | through | 3 | 3/3 | 33.9 | - |
| friction_patches | train | around | 5 | 4/5 | 44.5 | deadline x1 |
| friction_patches | train | through | 5 | 5/5 | 33.5 | - |
| lateral_slot | heldout | diffdrive | 3 | 0/3 | - | no_path x3 |
| lateral_slot | heldout | mecanum | 3 | 3/3 | 41.3 | - |
| lateral_slot | train | diffdrive | 5 | 0/5 | - | no_path x5 |
| lateral_slot | train | mecanum | 5 | 5/5 | 45.0 | - |
| long_payload_door | heldout | keep | 3 | 0/3 | - | no_path x3 |
| long_payload_door | heldout | reconfigure | 3 | 3/3 | 57.9 | - |
| long_payload_door | train | keep | 5 | 0/5 | - | no_path x5 |
| long_payload_door | train | reconfigure | 5 | 5/5 | 52.0 | - |
| narrow_door | heldout | couple_chain | 3 | 0/3 | - | no_path x3 |
| narrow_door | heldout | single | 3 | 3/3 | 56.0 | - |
| narrow_door | train | couple_chain | 5 | 0/5 | - | no_path x5 |
| narrow_door | train | single | 5 | 5/5 | 56.2 | - |
| wheel_failure | heldout | continue | 3 | 2/3 | 42.0 | could not reach x1 |
| wheel_failure | heldout | reassign | 3 | 3/3 | 75.5 | - |
| wheel_failure | train | continue | 5 | 2/5 | 45.9 | could not reach x3 |
| wheel_failure | train | reassign | 5 | 5/5 | 66.5 | - |

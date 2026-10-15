# Evaluation (yolo backend, 300 s per scenario, seed 42)

| Scenario | Parts | Pick success | Sorting acc. | Throughput [ppm] | Cycle time [s] | Class. acc. | Defect P / R | Failed grasps | Plan fails | Missed |
|---|---|---|---|---|---|---|---|---|---|---|
| nominal | 43 | 100.0 % | 100.0 % | 6.4 | 6.42 | 100.0 % | 100.0 % / 100.0 % | 0 | 0 | 0 |
| touching | 62 | 100.0 % | 97.0 % | 6.38 | 6.31 | 97.0 % | 100.0 % / 100.0 % | 0 | 0 | 0 |
| new_variants | 44 | 100.0 % | 100.0 % | 6.38 | 6.43 | 100.0 % | 100.0 % / 100.0 % | 0 | 0 | 0 |
| dim_light | 43 | 100.0 % | 100.0 % | 6.2 | 6.56 | 100.0 % | 100.0 % / 100.0 % | 0 | 0 | 0 |
| fast_belt | 75 | 100.0 % | 94.4 % | 6.8 | 6.26 | 97.1 % | 100.0 % / 100.0 % | 0 | 0 | 10 |

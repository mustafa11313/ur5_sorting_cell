# Evaluation (classical backend, 300 s per scenario, seed 42)

| Scenario | Parts | Pick success | Sorting acc. | Throughput [ppm] | Cycle time [s] | Class. acc. | Defect P / R | Failed grasps | Plan fails | Missed |
|---|---|---|---|---|---|---|---|---|---|---|
| nominal | 43 | 100.0 % | 100.0 % | 6.4 | 6.38 | 100.0 % | 100.0 % / 100.0 % | 0 | 0 | 0 |
| touching | 62 | 100.0 % | 90.9 % | 6.0 | 6.2 | 90.9 % | 100.0 % / 100.0 % | 0 | 0 | 0 |
| new_variants | 44 | 100.0 % | 96.9 % | 6.2 | 6.31 | 96.9 % | 100.0 % / 100.0 % | 0 | 0 | 0 |
| dim_light | 12 | 0.0 % | n/a | 0.0 | n/a | n/a | n/a / n/a | 17 | 1 | 0 |
| fast_belt | 74 | 56.9 % | 79.3 % | 4.66 | 6.16 | 79.3 % | 83.3 % / 83.3 % | 22 | 9 | 1 |

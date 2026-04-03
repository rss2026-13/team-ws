# PF quantitative testing summary

- Confidence metric: spread = sqrt(var(x) + var(y)); source = `/pf/particles` when available, otherwise rolling `/pf/pose/odom` jitter fallback
- Converged when spread <= 0.350 for at least 1.50s (starting search after 2.00s)
- Publishing rate from `/pf/pose/odom` timestamps

| bag | run | conf_source | converged | conv_time_s | post_conv_conf | min_conf | pub_rate_hz | pf_pose_msgs | pf_particles_msgs |
|-----|-----|-------------|-----------|-------------|----------------|----------|-------------|--------------|-------------------|
| corridor1 | 1 | particles | True | 2.019286 | 0.131989 | 0.071563 | 90.103280 | 1795 | 798 |
| corridor1 | 2 | odom_jitter | True | 1.986114 | 0.042092 | 0.011241 | 50.024718 | 942 | 0 |
| corridor1 | 3 | odom_jitter | True | 1.986462 | 0.027511 | 0.005651 | 50.009914 | 942 | 0 |
| corridor2 | 1 | odom_jitter | True | 1.980667 | 0.033251 | 0.002639 | 50.041244 | 616 | 0 |
| corridor2 | 2 | odom_jitter | True | 1.959924 | 0.027571 | 0.003004 | 49.989706 | 615 | 0 |
| corridor2 | 3 | odom_jitter | True | 2.001507 | 0.019450 | 0.005917 | 50.099933 | 617 | 0 |
| corridor3 | 1 | odom_jitter | True | 1.996941 | 0.024339 | 0.003369 | 50.034277 | 1371 | 0 |
| corridor3 | 2 | odom_jitter | True | 1.979670 | 0.025453 | 0.001810 | 49.986785 | 1372 | 0 |
| corridor3 | 3 | particles | True | 2.017992 | 0.141179 | 0.058469 | 88.115439 | 2543 | 1154 |
| corridor4 | 1 | odom_jitter | True | 2.004122 | 0.022690 | 0.003607 | 50.049702 | 1020 | 0 |
| corridor4 | 2 | odom_jitter | True | 1.979600 | 0.020940 | 0.003239 | 50.024438 | 1018 | 0 |
| corridor4 | 3 | odom_jitter | True | 2.002426 | 0.037596 | 0.004016 | 50.021247 | 1018 | 0 |

## Per-bag averages across runs

| bag | runs | converged_runs | avg_conv_time_s | avg_post_conv_conf | avg_publish_rate_hz |
|-----|------|----------------|-----------------|--------------------|---------------------|
| corridor1 | 3 | 3 | 1.997287 | 0.067197 | 63.379304 |
| corridor2 | 3 | 3 | 1.980700 | 0.026757 | 50.043628 |
| corridor3 | 3 | 3 | 1.998201 | 0.063657 | 62.712167 |
| corridor4 | 3 | 3 | 1.995382 | 0.027076 | 50.031796 |

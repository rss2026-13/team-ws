# PF quantitative testing summary

- Confidence metric: spread = sqrt(var(x) + var(y)); source = `/pf/particles` when available, otherwise rolling `/pf/pose/odom` jitter fallback
- Converged when spread <= 0.350 for at least 1.50s (starting search after 2.00s)
- Publishing rate from `/pf/pose/odom` timestamps

| bag | run | conf_source | converged | conv_time_s | post_conv_conf | min_conf | pub_rate_hz | pf_pose_msgs | pf_particles_msgs |
|-----|-----|-------------|-----------|-------------|----------------|----------|-------------|--------------|-------------------|
| corridor1 | 1 | odom_jitter | True | 1.998803 | 0.003666 | 0.000876 | 50.052540 | 938 | 0 |
| corridor1 | 2 | odom_jitter | True | 1.978576 | 0.004044 | 0.001099 | 50.037493 | 909 | 0 |
| corridor1 | 3 | odom_jitter | True | 2.004082 | 0.002667 | 0.000874 | 50.064611 | 942 | 0 |
| corridor2 | 1 | odom_jitter | True | 1.980727 | 0.004973 | 0.000820 | 50.049698 | 618 | 0 |
| corridor2 | 2 | odom_jitter | True | 1.979836 | 0.003639 | 0.000454 | 50.051910 | 618 | 0 |
| corridor2 | 3 | odom_jitter | True | 1.977290 | 0.005647 | 0.000231 | 50.083543 | 616 | 0 |
| corridor3 | 1 | odom_jitter | True | 2.003223 | 0.004982 | 0.000575 | 50.040851 | 1364 | 0 |
| corridor3 | 2 | odom_jitter | True | 2.001245 | 0.002252 | 0.000656 | 50.026927 | 1364 | 0 |
| corridor3 | 3 | odom_jitter | True | 1.978088 | 0.007169 | 0.000563 | 50.015786 | 1363 | 0 |
| corridor4 | 1 | odom_jitter | True | 1.976074 | 0.004247 | 0.000442 | 50.019590 | 1019 | 0 |
| corridor4 | 2 | odom_jitter | True | 1.980467 | 0.004274 | 0.000138 | 50.020050 | 1020 | 0 |
| corridor4 | 3 | odom_jitter | True | 1.981392 | 0.005731 | 0.000410 | 50.027886 | 1020 | 0 |

## Per-bag averages across runs

| bag | runs | converged_runs | avg_conv_time_s | avg_post_conv_conf | avg_publish_rate_hz |
|-----|------|----------------|-----------------|--------------------|---------------------|
| corridor1 | 3 | 3 | 1.993821 | 0.003459 | 50.051548 |
| corridor2 | 3 | 3 | 1.979285 | 0.004753 | 50.061717 |
| corridor3 | 3 | 3 | 1.994185 | 0.004801 | 50.027854 |
| corridor4 | 3 | 3 | 1.979311 | 0.004751 | 50.022509 |

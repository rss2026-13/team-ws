# PF quantitative testing summary

- Confidence metric: spread = sqrt(var(x) + var(y)); source = `/pf/particles` when available, otherwise rolling `/pf/pose/odom` jitter fallback
- Converged when spread <= 0.350 for at least 1.50s (starting search after 2.00s)
- Publishing rate from `/pf/pose/odom` timestamps

| bag | run | conf_source | converged | conv_time_s | post_conv_conf | min_conf | pub_rate_hz | pf_pose_msgs | pf_particles_msgs |
|-----|-----|-------------|-----------|-------------|----------------|----------|-------------|--------------|-------------------|
| corridor1 | 1 | odom_jitter | True | 1.973639 | 0.023874 | 0.002897 | 50.011550 | 958 | 0 |
| corridor1 | 2 | odom_jitter | True | 1.979616 | 0.028706 | 0.001671 | 50.027074 | 959 | 0 |
| corridor1 | 3 | particles | True | 2.017146 | 0.122138 | 0.074692 | 90.110314 | 1795 | 798 |
| corridor2 | 1 | odom_jitter | True | 1.978251 | 0.032548 | 0.002812 | 50.069249 | 617 | 0 |
| corridor2 | 2 | odom_jitter | True | 1.986867 | 0.015812 | 0.001855 | 49.998681 | 617 | 0 |
| corridor2 | 3 | odom_jitter | True | 1.978782 | 0.027091 | 0.002488 | 50.056019 | 617 | 0 |
| corridor3 | 1 | odom_jitter | True | 1.977241 | 0.021218 | 0.002193 | 50.010958 | 1372 | 0 |
| corridor3 | 2 | odom_jitter | True | 1.979978 | 0.015806 | 0.001760 | 50.022133 | 1364 | 0 |
| corridor3 | 3 | odom_jitter | True | 1.982494 | 0.020037 | 0.002209 | 49.984006 | 1362 | 0 |
| corridor4 | 1 | particles | True | 2.012216 | 0.150692 | 0.051595 | 90.055084 | 1934 | 861 |
| corridor4 | 2 | odom_jitter | True | 1.974838 | 0.018182 | 0.001675 | 50.026131 | 1019 | 0 |
| corridor4 | 3 | odom_jitter | True | 1.958678 | 0.032029 | 0.002906 | 49.990174 | 1018 | 0 |

## Per-bag averages across runs

| bag | runs | converged_runs | avg_conv_time_s | avg_post_conv_conf | avg_publish_rate_hz |
|-----|------|----------------|-----------------|--------------------|---------------------|
| corridor1 | 3 | 3 | 1.990134 | 0.058240 | 63.382979 |
| corridor2 | 3 | 3 | 1.981300 | 0.025150 | 50.041316 |
| corridor3 | 3 | 3 | 1.979904 | 0.019020 | 50.005699 |
| corridor4 | 3 | 3 | 1.981911 | 0.066968 | 63.357130 |

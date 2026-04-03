# PF quantitative testing summary

- Confidence metric: spread = sqrt(var(x) + var(y)); source = `/pf/particles` when available, otherwise rolling `/pf/pose/odom` jitter fallback
- Converged when spread <= 0.350 for at least 1.50s (starting search after 2.00s)
- Publishing rate from `/pf/pose/odom` timestamps

| bag | run | conf_source | converged | conv_time_s | post_conv_conf | min_conf | pub_rate_hz | pf_pose_msgs | pf_particles_msgs |
|-----|-----|-------------|-----------|-------------|----------------|----------|-------------|--------------|-------------------|
| corridor1 | 1 | odom_jitter | True | 2.000715 | 0.025732 | 0.002107 | 50.053034 | 960 | 0 |
| corridor1 | 2 | odom_jitter | True | 1.984716 | 0.027447 | 0.004318 | 50.040714 | 942 | 0 |
| corridor1 | 3 | odom_jitter | True | 1.975020 | 0.014294 | 0.003913 | 50.032499 | 925 | 0 |
| corridor2 | 1 | particles | True | 2.000318 | 0.095649 | 0.058612 | 90.184707 | 1212 | 539 |
| corridor2 | 2 | odom_jitter | True | 1.976359 | 0.010998 | 0.001885 | 50.049443 | 617 | 0 |
| corridor2 | 3 | odom_jitter | True | 1.975822 | 0.022041 | 0.001014 | 50.051887 | 617 | 0 |
| corridor3 | 1 | odom_jitter | True | 1.975834 | 0.014556 | 0.001983 | 50.017990 | 1365 | 0 |
| corridor3 | 2 | odom_jitter | True | 1.979497 | 0.019919 | 0.003106 | 50.035053 | 1363 | 0 |
| corridor3 | 3 | odom_jitter | True | 1.978668 | 0.014179 | 0.001179 | 50.019470 | 1372 | 0 |
| corridor4 | 1 | odom_jitter | True | 1.979434 | 0.028349 | 0.001314 | 50.042125 | 1019 | 0 |
| corridor4 | 2 | odom_jitter | True | 1.976852 | 0.020500 | 0.001697 | 50.011943 | 1021 | 0 |
| corridor4 | 3 | odom_jitter | True | 1.980859 | 0.019568 | 0.002206 | 50.031119 | 1019 | 0 |

## Per-bag averages across runs

| bag | runs | converged_runs | avg_conv_time_s | avg_post_conv_conf | avg_publish_rate_hz |
|-----|------|----------------|-----------------|--------------------|---------------------|
| corridor1 | 3 | 3 | 1.986817 | 0.022491 | 50.042082 |
| corridor2 | 3 | 3 | 1.984166 | 0.042896 | 63.428679 |
| corridor3 | 3 | 3 | 1.978000 | 0.016218 | 50.024171 |
| corridor4 | 3 | 3 | 1.979048 | 0.022806 | 50.028396 |

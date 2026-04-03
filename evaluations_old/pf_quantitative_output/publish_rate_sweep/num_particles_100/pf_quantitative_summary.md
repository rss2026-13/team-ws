# PF quantitative testing summary

- Confidence metric: spread = sqrt(var(x) + var(y)); source = `/pf/particles` when available, otherwise rolling `/pf/pose/odom` jitter fallback
- Converged when spread <= 0.350 for at least 1.50s (starting search after 2.00s)
- Publishing rate from `/pf/pose/odom` timestamps

| bag | run | conf_source | converged | conv_time_s | post_conv_conf | min_conf | pub_rate_hz | pf_pose_msgs | pf_particles_msgs |
|-----|-----|-------------|-----------|-------------|----------------|----------|-------------|--------------|-------------------|
| corridor1 | 1 | particles | True | 2.020175 | 0.098790 | 0.056508 | 90.042903 | 1794 | 798 |
| corridor1 | 2 | odom_jitter | True | 1.980201 | 0.022160 | 0.004905 | 50.042189 | 926 | 0 |
| corridor1 | 3 | odom_jitter | True | 1.976197 | 0.010589 | 0.003277 | 49.997404 | 958 | 0 |
| corridor2 | 1 | particles | True | 2.001885 | 0.098583 | 0.059713 | 89.885923 | 1235 | 550 |
| corridor2 | 2 | odom_jitter | True | 1.980359 | 0.013765 | 0.001681 | 50.028110 | 616 | 0 |
| corridor2 | 3 | particles | True | 2.005203 | 0.096447 | 0.060563 | 90.216929 | 1209 | 537 |
| corridor3 | 1 | odom_jitter | True | 1.980116 | 0.021779 | 0.000756 | 50.006527 | 1370 | 0 |
| corridor3 | 2 | odom_jitter | True | 2.004254 | 0.024877 | 0.000968 | 49.987984 | 1370 | 0 |
| corridor3 | 3 | odom_jitter | True | 1.977247 | 0.012638 | 0.001739 | 50.020268 | 1371 | 0 |
| corridor4 | 1 | odom_jitter | True | 1.978263 | 0.014900 | 0.003494 | 49.998413 | 1018 | 0 |
| corridor4 | 2 | odom_jitter | True | 1.986979 | 0.030809 | 0.001684 | 50.049118 | 1017 | 0 |
| corridor4 | 3 | odom_jitter | True | 1.980665 | 0.029499 | 0.002248 | 50.043724 | 1022 | 0 |

## Per-bag averages across runs

| bag | runs | converged_runs | avg_conv_time_s | avg_post_conv_conf | avg_publish_rate_hz |
|-----|------|----------------|-----------------|--------------------|---------------------|
| corridor1 | 3 | 3 | 1.992191 | 0.043846 | 63.360832 |
| corridor2 | 3 | 3 | 1.995816 | 0.069598 | 76.710321 |
| corridor3 | 3 | 3 | 1.987206 | 0.019765 | 50.004926 |
| corridor4 | 3 | 3 | 1.981969 | 0.025069 | 50.030418 |

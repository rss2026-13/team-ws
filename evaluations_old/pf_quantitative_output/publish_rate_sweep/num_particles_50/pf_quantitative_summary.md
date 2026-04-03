# PF quantitative testing summary

- Confidence metric: spread = sqrt(var(x) + var(y)); source = `/pf/particles` when available, otherwise rolling `/pf/pose/odom` jitter fallback
- Converged when spread <= 0.350 for at least 1.50s (starting search after 2.00s)
- Publishing rate from `/pf/pose/odom` timestamps

| bag | run | conf_source | converged | conv_time_s | post_conv_conf | min_conf | pub_rate_hz | pf_pose_msgs | pf_particles_msgs |
|-----|-----|-------------|-----------|-------------|----------------|----------|-------------|--------------|-------------------|
| corridor1 | 1 | particles | True | 2.013225 | 0.095041 | 0.046984 | 90.007950 | 1793 | 797 |
| corridor1 | 2 | odom_jitter | True | 1.977460 | 0.034272 | 0.002257 | 50.037724 | 959 | 0 |
| corridor1 | 3 | odom_jitter | True | 2.007067 | 0.017616 | 0.006469 | 49.998093 | 941 | 0 |
| corridor2 | 1 | odom_jitter | True | 1.975687 | 0.037006 | 0.001766 | 50.000850 | 616 | 0 |
| corridor2 | 2 | odom_jitter | True | 1.977331 | 0.019362 | 0.002972 | 50.072159 | 616 | 0 |
| corridor2 | 3 | odom_jitter | True | 1.999641 | 0.045448 | 0.003274 | 50.087814 | 617 | 0 |
| corridor3 | 1 | odom_jitter | True | 1.980512 | 0.026753 | 0.001399 | 50.019072 | 1371 | 0 |
| corridor3 | 2 | odom_jitter | True | 1.993709 | 0.036286 | 0.000994 | 49.998410 | 1371 | 0 |
| corridor3 | 3 | odom_jitter | True | 1.978309 | 0.017742 | 0.002918 | 50.035111 | 1372 | 0 |
| corridor4 | 1 | odom_jitter | True | 2.001204 | 0.029343 | 0.002767 | 50.036875 | 1018 | 0 |
| corridor4 | 2 | odom_jitter | True | 1.980101 | 0.039277 | 0.003731 | 50.043508 | 1019 | 0 |
| corridor4 | 3 | particles | True | 2.004606 | 0.114539 | 0.047791 | 90.159215 | 1964 | 873 |

## Per-bag averages across runs

| bag | runs | converged_runs | avg_conv_time_s | avg_post_conv_conf | avg_publish_rate_hz |
|-----|------|----------------|-----------------|--------------------|---------------------|
| corridor1 | 3 | 3 | 1.999251 | 0.048976 | 63.347922 |
| corridor2 | 3 | 3 | 1.984220 | 0.033939 | 50.053608 |
| corridor3 | 3 | 3 | 1.984177 | 0.026927 | 50.017531 |
| corridor4 | 3 | 3 | 1.995304 | 0.061053 | 63.413199 |

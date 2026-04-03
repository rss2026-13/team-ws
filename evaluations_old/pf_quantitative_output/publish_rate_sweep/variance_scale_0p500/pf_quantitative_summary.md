# PF quantitative testing summary

- Confidence metric: spread = sqrt(var(x) + var(y)); source = `/pf/particles` when available, otherwise rolling `/pf/pose/odom` jitter fallback
- Converged when spread <= 0.350 for at least 1.50s (starting search after 2.00s)
- Publishing rate from `/pf/pose/odom` timestamps

| bag | run | conf_source | converged | conv_time_s | post_conv_conf | min_conf | pub_rate_hz | pf_pose_msgs | pf_particles_msgs |
|-----|-----|-------------|-----------|-------------|----------------|----------|-------------|--------------|-------------------|
| corridor1 | 1 | particles | True | 2.000199 | 0.098758 | 0.052866 | 316.451693 | 6305 | 2389 |
| corridor1 | 2 | particles | True | 2.001486 | 0.106103 | 0.044722 | 316.888211 | 6313 | 2364 |
| corridor2 | 1 | odom_jitter | True | 1.979634 | 0.020579 | 0.001285 | 50.050348 | 616 | 0 |
| corridor2 | 2 | odom_jitter | True | 1.979773 | 0.016973 | 0.000484 | 50.072944 | 617 | 0 |
| corridor2 | 3 | odom_jitter | True | 1.980012 | 0.020627 | 0.000814 | 50.089523 | 616 | 0 |
| corridor3 | 1 | odom_jitter | True | 1.973886 | 0.011357 | 0.000964 | 49.978450 | 1371 | 0 |
| corridor3 | 2 | particles | True | 2.019211 | 0.455412 | 0.047638 | 90.100726 | 2564 | 1140 |
| corridor3 | 3 | particles | True | 2.020902 | 0.694311 | 0.039299 | 89.914950 | 2563 | 1140 |
| corridor4 | 1 | odom_jitter | True | 1.980080 | 0.023843 | 0.000732 | 50.020172 | 1018 | 0 |
| corridor4 | 2 | odom_jitter | True | 1.980875 | 0.012771 | 0.001162 | 50.006058 | 1018 | 0 |
| corridor4 | 3 | odom_jitter | True | 1.981945 | 0.024220 | 0.001182 | 50.040785 | 1019 | 0 |

## Per-bag averages across runs

| bag | runs | converged_runs | avg_conv_time_s | avg_post_conv_conf | avg_publish_rate_hz |
|-----|------|----------------|-----------------|--------------------|---------------------|
| corridor1 | 2 | 2 | 2.000842 | 0.102431 | 316.669952 |
| corridor2 | 3 | 3 | 1.979806 | 0.019393 | 50.070938 |
| corridor3 | 3 | 3 | 2.004666 | 0.387027 | 76.664709 |
| corridor4 | 3 | 3 | 1.980967 | 0.020278 | 50.022338 |

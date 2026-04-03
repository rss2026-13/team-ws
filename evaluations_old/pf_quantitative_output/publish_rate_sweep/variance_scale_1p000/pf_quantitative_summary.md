# PF quantitative testing summary

- Confidence metric: spread = sqrt(var(x) + var(y)); source = `/pf/particles` when available, otherwise rolling `/pf/pose/odom` jitter fallback
- Converged when spread <= 0.350 for at least 1.50s (starting search after 2.00s)
- Publishing rate from `/pf/pose/odom` timestamps

| bag | run | conf_source | converged | conv_time_s | post_conv_conf | min_conf | pub_rate_hz | pf_pose_msgs | pf_particles_msgs |
|-----|-----|-------------|-----------|-------------|----------------|----------|-------------|--------------|-------------------|
| corridor1 | 1 | odom_jitter | True | 1.998169 | 0.028946 | 0.006209 | 50.045725 | 925 | 0 |
| corridor1 | 2 | odom_jitter | True | 1.994848 | 0.051909 | 0.004931 | 50.050167 | 926 | 0 |
| corridor1 | 3 | odom_jitter | True | 1.979019 | 0.014787 | 0.005226 | 50.033647 | 926 | 0 |
| corridor2 | 1 | odom_jitter | True | 1.983289 | 0.020845 | 0.001075 | 50.026176 | 617 | 0 |
| corridor2 | 2 | odom_jitter | True | 1.980552 | 0.027966 | 0.002466 | 49.997286 | 616 | 0 |
| corridor2 | 3 | odom_jitter | True | 1.978176 | 0.021933 | 0.000493 | 50.054177 | 616 | 0 |
| corridor3 | 1 | odom_jitter | True | 1.975908 | 0.013320 | 0.000675 | 50.005197 | 1371 | 0 |
| corridor3 | 2 | particles | True | 2.015581 | 0.130901 | 0.056000 | 90.038562 | 2598 | 1155 |
| corridor3 | 3 | odom_jitter | True | 1.976818 | 0.011100 | 0.002555 | 49.974908 | 1368 | 0 |
| corridor4 | 1 | odom_jitter | True | 1.965810 | 0.030016 | 0.002133 | 50.010313 | 1019 | 0 |
| corridor4 | 2 | odom_jitter | True | 1.981339 | 0.034482 | 0.001763 | 50.014795 | 1019 | 0 |
| corridor4 | 3 | particles | True | 2.008250 | 0.133201 | 0.040342 | 90.030421 | 1901 | 846 |

## Per-bag averages across runs

| bag | runs | converged_runs | avg_conv_time_s | avg_post_conv_conf | avg_publish_rate_hz |
|-----|------|----------------|-----------------|--------------------|---------------------|
| corridor1 | 3 | 3 | 1.990679 | 0.031881 | 50.043179 |
| corridor2 | 3 | 3 | 1.980673 | 0.023581 | 50.025880 |
| corridor3 | 3 | 3 | 1.989436 | 0.051774 | 63.339556 |
| corridor4 | 3 | 3 | 1.985133 | 0.065900 | 63.351843 |

# PF quantitative testing summary

- Confidence metric: spread = sqrt(var(x) + var(y)); source = `/pf/particles` when available, otherwise rolling `/pf/pose/odom` jitter fallback
- Converged when spread <= 0.350 for at least 1.50s (starting search after 2.00s)
- Publishing rate from `/pf/pose/odom` timestamps

| bag | run | conf_source | converged | conv_time_s | post_conv_conf | min_conf | pub_rate_hz | pf_pose_msgs | pf_particles_msgs |
|-----|-----|-------------|-----------|-------------|----------------|----------|-------------|--------------|-------------------|
| corridor1 | 1 | odom_jitter | True | 1.981556 | 0.018060 | 0.003127 | 50.019589 | 959 | 0 |
| corridor1 | 2 | odom_jitter | True | 1.973991 | 0.016850 | 0.004600 | 50.001683 | 925 | 0 |
| corridor1 | 3 | odom_jitter | True | 1.982949 | 0.025470 | 0.006178 | 50.002920 | 942 | 0 |
| corridor2 | 1 | odom_jitter | True | 1.981135 | 0.015312 | 0.001628 | 50.031965 | 616 | 0 |
| corridor2 | 2 | odom_jitter | True | 1.984738 | 0.019299 | 0.002123 | 50.074700 | 618 | 0 |
| corridor2 | 3 | odom_jitter | True | 1.977528 | 0.029757 | 0.002438 | 49.956918 | 616 | 0 |
| corridor3 | 1 | odom_jitter | True | 1.973840 | 0.014133 | 0.001163 | 50.007516 | 1363 | 0 |
| corridor3 | 2 | particles | True | 2.017119 | 0.111830 | 0.033052 | 89.871420 | 2546 | 1134 |
| corridor3 | 3 | odom_jitter | True | 1.993272 | 0.021417 | 0.001302 | 50.039108 | 1364 | 0 |
| corridor4 | 1 | odom_jitter | True | 1.981510 | 0.018941 | 0.002182 | 50.034003 | 1020 | 0 |
| corridor4 | 2 | odom_jitter | True | 1.980013 | 0.014512 | 0.002659 | 50.011789 | 1020 | 0 |
| corridor4 | 3 | odom_jitter | True | 1.976642 | 0.037663 | 0.002492 | 50.000886 | 1019 | 0 |

## Per-bag averages across runs

| bag | runs | converged_runs | avg_conv_time_s | avg_post_conv_conf | avg_publish_rate_hz |
|-----|------|----------------|-----------------|--------------------|---------------------|
| corridor1 | 3 | 3 | 1.979499 | 0.020127 | 50.008064 |
| corridor2 | 3 | 3 | 1.981133 | 0.021456 | 50.021194 |
| corridor3 | 3 | 3 | 1.994744 | 0.049127 | 63.306015 |
| corridor4 | 3 | 3 | 1.979389 | 0.023705 | 50.015559 |

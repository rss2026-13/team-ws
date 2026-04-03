# PF quantitative testing summary

- Confidence metric: spread = sqrt(var(x) + var(y)); source = `/pf/particles` when available, otherwise rolling `/pf/pose/odom` jitter fallback
- Converged when spread <= 0.350 for at least 1.50s (starting search after 2.00s)
- Publishing rate from `/pf/pose/odom` timestamps

| bag | run | conf_source | converged | conv_time_s | post_conv_conf | min_conf | pub_rate_hz | pf_pose_msgs | pf_particles_msgs |
|-----|-----|-------------|-----------|-------------|----------------|----------|-------------|--------------|-------------------|
| corridor1 | 1 | particles | True | 2.022380 | 0.114150 | 0.070686 | 89.865419 | 1707 | 759 |
| corridor1 | 2 | odom_jitter | True | 1.983277 | 0.011719 | 0.003832 | 50.014852 | 942 | 0 |
| corridor1 | 3 | odom_jitter | True | 1.985616 | 0.022856 | 0.002790 | 50.026346 | 942 | 0 |
| corridor2 | 1 | odom_jitter | True | 1.980656 | 0.014670 | 0.001873 | 50.034572 | 618 | 0 |
| corridor2 | 2 | particles | True | 2.000717 | 0.101773 | 0.068217 | 90.119407 | 1240 | 552 |
| corridor2 | 3 | odom_jitter | True | 2.008891 | 0.012185 | 0.001195 | 50.074020 | 618 | 0 |
| corridor3 | 1 | odom_jitter | True | 1.977266 | 0.012675 | 0.001100 | 50.022720 | 1372 | 0 |
| corridor3 | 2 | odom_jitter | True | 1.979916 | 0.010362 | 0.001836 | 50.042388 | 1363 | 0 |
| corridor3 | 3 | odom_jitter | True | 1.979016 | 0.012166 | 0.001997 | 50.025654 | 1365 | 0 |
| corridor4 | 1 | odom_jitter | True | 2.010496 | 0.029716 | 0.002024 | 50.035072 | 1020 | 0 |
| corridor4 | 2 | odom_jitter | True | 1.979972 | 0.012016 | 0.000984 | 50.017319 | 1020 | 0 |
| corridor4 | 3 | odom_jitter | True | 1.979868 | 0.009655 | 0.000817 | 50.024899 | 1020 | 0 |

## Per-bag averages across runs

| bag | runs | converged_runs | avg_conv_time_s | avg_post_conv_conf | avg_publish_rate_hz |
|-----|------|----------------|-----------------|--------------------|---------------------|
| corridor1 | 3 | 3 | 1.997091 | 0.049575 | 63.302206 |
| corridor2 | 3 | 3 | 1.996755 | 0.042876 | 63.409333 |
| corridor3 | 3 | 3 | 1.978733 | 0.011734 | 50.030254 |
| corridor4 | 3 | 3 | 1.990112 | 0.017129 | 50.025763 |

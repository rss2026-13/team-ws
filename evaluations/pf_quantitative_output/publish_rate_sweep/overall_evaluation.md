# Overall PF setting evaluation

Ranking criteria (in order):
1. Higher endpoint success rate
2. Lower mean post-convergence spread
3. Higher mean publish rate

## Best overall setting

- Setting: `sigma_x_only_x1p000_y0p100_t0p080`
- Endpoint success: 11/12 (0.917)
- Mean post-convergence spread: n/a
- Mean publish rate: 66.924 Hz

### What this setting means

- `num_particles = 2000`
- `num_beams_per_particle = 10`
- `sigma_x = 1.000`
- `sigma_y = 0.100`
- `sigma_theta = 0.080`

## Best setting per family

| family | setting | success_rate | mean_post_conf | mean_publish_rate_hz | particles | rays | sigma_x | sigma_y | sigma_theta |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| pr_combo | pr_combo_p200_r150 | 0.750 | 0.109 | 89.871 | 200 | 150 | 0.030 | 0.010 | 0.040 |
| sigma_t_only | sigma_t_only_x0p100_y0p100_t0p040 | 0.417 | 0.313 | 69.165 | 2000 | 10 | 0.100 | 0.100 | 0.040 |
| sigma_x_only | sigma_x_only_x1p000_y0p100_t0p080 | 0.917 | n/a | 66.924 | 2000 | 10 | 1.000 | 0.100 | 0.080 |
| sigma_y_only | sigma_y_only_x0p100_y1p000_t0p080 | 0.583 | n/a | 70.194 | 2000 | 10 | 0.100 | 1.000 | 0.080 |

## Top 10 settings overall

| rank | setting | success_rate | mean_post_conf | mean_publish_rate_hz | particles | rays | sigma_x | sigma_y | sigma_theta |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | sigma_x_only_x1p000_y0p100_t0p080 | 0.917 | n/a | 66.924 | 2000 | 10 | 1.000 | 0.100 | 0.080 |
| 2 | pr_combo_p200_r150 | 0.750 | 0.109 | 89.871 | 200 | 150 | 0.030 | 0.010 | 0.040 |
| 3 | pr_combo_p50_r150 | 0.667 | 0.112 | 90.097 | 50 | 150 | 0.030 | 0.010 | 0.040 |
| 4 | pr_combo_p100_r150 | 0.583 | 0.111 | 90.041 | 100 | 150 | 0.030 | 0.010 | 0.040 |
| 5 | sigma_y_only_x0p100_y1p000_t0p080 | 0.583 | n/a | 70.194 | 2000 | 10 | 0.100 | 1.000 | 0.080 |
| 6 | pr_combo_p100_r99 | 0.500 | 0.121 | 90.095 | 100 | 99 | 0.030 | 0.010 | 0.040 |
| 7 | pr_combo_p200_r99 | 0.500 | 0.125 | 90.035 | 200 | 99 | 0.030 | 0.010 | 0.040 |
| 8 | sigma_y_only_x0p100_y0p100_t0p080 | 0.500 | 0.318 | 68.422 | 2000 | 10 | 0.100 | 0.100 | 0.080 |
| 9 | sigma_t_only_x0p100_y0p100_t0p040 | 0.417 | 0.313 | 69.165 | 2000 | 10 | 0.100 | 0.100 | 0.040 |
| 10 | sigma_x_only_x0p100_y0p100_t0p080 | 0.417 | 0.323 | 68.459 | 2000 | 10 | 0.100 | 0.100 | 0.080 |

## Top 10 settings for `corridor1`

| rank | setting | success_rate | mean_post_conf | mean_publish_rate_hz | particles | rays | sigma_x | sigma_y | sigma_theta |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | pr_combo_p50_r150 | 1.000 | 0.101 | 90.093 | 50 | 150 | 0.030 | 0.010 | 0.040 |
| 2 | pr_combo_p200_r150 | 1.000 | 0.104 | 89.898 | 200 | 150 | 0.030 | 0.010 | 0.040 |
| 3 | pr_combo_p100_r150 | 1.000 | 0.104 | 90.052 | 100 | 150 | 0.030 | 0.010 | 0.040 |
| 4 | pr_combo_p2000_r10 | 1.000 | 0.231 | 68.187 | 2000 | 10 | 0.030 | 0.010 | 0.040 |
| 5 | sigma_y_only_x0p100_y0p050_t0p080 | 1.000 | 0.337 | 68.112 | 2000 | 10 | 0.100 | 0.050 | 0.080 |
| 6 | sigma_y_only_x0p100_y1p000_t0p080 | 1.000 | n/a | 69.797 | 2000 | 10 | 0.100 | 1.000 | 0.080 |
| 7 | sigma_y_only_x0p100_y0p100_t0p080 | 1.000 | n/a | 68.275 | 2000 | 10 | 0.100 | 0.100 | 0.080 |
| 8 | sigma_t_only_x0p100_y0p100_t0p040 | 1.000 | n/a | 67.665 | 2000 | 10 | 0.100 | 0.100 | 0.040 |
| 9 | sigma_x_only_x1p000_y0p100_t0p080 | 1.000 | n/a | 67.376 | 2000 | 10 | 1.000 | 0.100 | 0.080 |
| 10 | sigma_t_only_x0p100_y0p100_t0p020 | 1.000 | n/a | 67.366 | 2000 | 10 | 0.100 | 0.100 | 0.020 |

## Top 10 settings for `corridor2`

| rank | setting | success_rate | mean_post_conf | mean_publish_rate_hz | particles | rays | sigma_x | sigma_y | sigma_theta |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | pr_combo_p100_r99 | 1.000 | 0.104 | 90.204 | 100 | 99 | 0.030 | 0.010 | 0.040 |
| 2 | pr_combo_p200_r99 | 1.000 | 0.108 | 90.159 | 200 | 99 | 0.030 | 0.010 | 0.040 |
| 3 | pr_combo_p100_r150 | 0.667 | 0.099 | 90.149 | 100 | 150 | 0.030 | 0.010 | 0.040 |
| 4 | pr_combo_p200_r150 | 0.667 | 0.103 | 89.941 | 200 | 150 | 0.030 | 0.010 | 0.040 |
| 5 | pr_combo_p50_r150 | 0.667 | 0.132 | 90.125 | 50 | 150 | 0.030 | 0.010 | 0.040 |
| 6 | sigma_x_only_x1p000_y0p100_t0p080 | 0.667 | n/a | 64.564 | 2000 | 10 | 1.000 | 0.100 | 0.080 |
| 7 | sigma_y_only_x0p100_y0p100_t0p080 | 0.333 | 0.334 | 66.448 | 2000 | 10 | 0.100 | 0.100 | 0.080 |
| 8 | sigma_y_only_x0p100_y0p050_t0p080 | 0.333 | 0.384 | 64.640 | 2000 | 10 | 0.100 | 0.050 | 0.080 |
| 9 | sigma_y_only_x0p100_y1p000_t0p080 | 0.333 | n/a | 67.778 | 2000 | 10 | 0.100 | 1.000 | 0.080 |
| 10 | pr_combo_p50_r99 | 0.000 | 0.099 | 90.223 | 50 | 99 | 0.030 | 0.010 | 0.040 |

## Top 10 settings for `corridor3`

| rank | setting | success_rate | mean_post_conf | mean_publish_rate_hz | particles | rays | sigma_x | sigma_y | sigma_theta |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | sigma_y_only_x0p100_y0p010_t0p080 | 1.000 | 0.433 | 67.778 | 2000 | 10 | 0.100 | 0.010 | 0.080 |
| 2 | sigma_y_only_x0p100_y1p000_t0p080 | 1.000 | n/a | 71.521 | 2000 | 10 | 0.100 | 1.000 | 0.080 |
| 3 | sigma_x_only_x1p000_y0p100_t0p080 | 1.000 | n/a | 66.550 | 2000 | 10 | 1.000 | 0.100 | 0.080 |
| 4 | sigma_t_only_x0p100_y0p100_t0p040 | 0.667 | 0.309 | 69.872 | 2000 | 10 | 0.100 | 0.100 | 0.040 |
| 5 | sigma_y_only_x0p100_y0p100_t0p080 | 0.667 | 0.310 | 67.022 | 2000 | 10 | 0.100 | 0.100 | 0.080 |
| 6 | sigma_x_only_x0p100_y0p100_t0p080 | 0.667 | 0.310 | 68.997 | 2000 | 10 | 0.100 | 0.100 | 0.080 |
| 7 | pr_combo_p50_r150 | 0.333 | 0.102 | 90.061 | 50 | 150 | 0.030 | 0.010 | 0.040 |
| 8 | pr_combo_p200_r150 | 0.333 | 0.111 | 89.687 | 200 | 150 | 0.030 | 0.010 | 0.040 |
| 9 | pr_combo_p100_r150 | 0.333 | 0.117 | 89.888 | 100 | 150 | 0.030 | 0.010 | 0.040 |
| 10 | pr_combo_p200_r99 | 0.333 | 0.130 | 89.852 | 200 | 99 | 0.030 | 0.010 | 0.040 |

## Top 10 settings for `corridor4`

| rank | setting | success_rate | mean_post_conf | mean_publish_rate_hz | particles | rays | sigma_x | sigma_y | sigma_theta |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | pr_combo_p200_r150 | 1.000 | 0.120 | 89.959 | 200 | 150 | 0.030 | 0.010 | 0.040 |
| 2 | sigma_x_only_x1p000_y0p100_t0p080 | 1.000 | n/a | 69.207 | 2000 | 10 | 1.000 | 0.100 | 0.080 |
| 3 | pr_combo_p50_r150 | 0.667 | 0.114 | 90.110 | 50 | 150 | 0.030 | 0.010 | 0.040 |
| 4 | pr_combo_p100_r150 | 0.333 | 0.126 | 90.077 | 100 | 150 | 0.030 | 0.010 | 0.040 |
| 5 | pr_combo_p100_r99 | 0.333 | 0.140 | 90.111 | 100 | 99 | 0.030 | 0.010 | 0.040 |
| 6 | pr_combo_p50_r99 | 0.333 | 0.144 | 90.136 | 50 | 99 | 0.030 | 0.010 | 0.040 |
| 7 | pr_combo_p200_r99 | 0.000 | 0.149 | 90.085 | 200 | 99 | 0.030 | 0.010 | 0.040 |
| 8 | pr_combo_p2000_r10 | 0.000 | 0.291 | 69.745 | 2000 | 10 | 0.030 | 0.010 | 0.040 |
| 9 | sigma_x_only_x0p010_y0p100_t0p080 | 0.000 | 0.317 | 68.847 | 2000 | 10 | 0.010 | 0.100 | 0.080 |
| 10 | sigma_y_only_x0p100_y0p010_t0p080 | 0.000 | 0.326 | 68.157 | 2000 | 10 | 0.100 | 0.010 | 0.080 |

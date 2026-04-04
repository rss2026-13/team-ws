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

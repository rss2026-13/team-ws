# Overall PF setting evaluation

Ranking criteria (in order):
1. Higher endpoint success rate
2. Lower mean final error
3. Lower mean post-convergence spread
4. Higher mean publish rate

## Best overall setting

- Setting: `sigma_x_only_x1p000_y0p100_t0p080`
- Endpoint success: 11/12 (0.917)
- Mean final error: 3.503 m
- Mean post-convergence spread: n/a
- Mean publish rate: 66.924 Hz

## Best setting per family

| family | setting | success_rate | mean_final_error_m | mean_post_conf | mean_publish_rate_hz |
|---|---|---:|---:|---:|---:|
| pr_combo | pr_combo_p200_r150 | 0.750 | 4.744 | 0.109 | 89.871 |
| sigma_t_only | sigma_t_only_x0p100_y0p100_t0p040 | 0.417 | 9.840 | 0.313 | 69.165 |
| sigma_x_only | sigma_x_only_x1p000_y0p100_t0p080 | 0.917 | 3.503 | n/a | 66.924 |
| sigma_y_only | sigma_y_only_x0p100_y1p000_t0p080 | 0.583 | 7.592 | n/a | 70.194 |

## Top 10 settings overall

| rank | setting | success_rate | mean_final_error_m | mean_post_conf | mean_publish_rate_hz |
|---:|---|---:|---:|---:|---:|
| 1 | sigma_x_only_x1p000_y0p100_t0p080 | 0.917 | 3.503 | n/a | 66.924 |
| 2 | pr_combo_p200_r150 | 0.750 | 4.744 | 0.109 | 89.871 |
| 3 | pr_combo_p50_r150 | 0.667 | 5.573 | 0.112 | 90.097 |
| 4 | pr_combo_p100_r150 | 0.583 | 7.208 | 0.111 | 90.041 |
| 5 | sigma_y_only_x0p100_y1p000_t0p080 | 0.583 | 7.592 | n/a | 70.194 |
| 6 | pr_combo_p100_r99 | 0.500 | 8.361 | 0.121 | 90.095 |
| 7 | sigma_y_only_x0p100_y0p100_t0p080 | 0.500 | 8.756 | 0.318 | 68.422 |
| 8 | pr_combo_p200_r99 | 0.500 | 9.520 | 0.125 | 90.035 |
| 9 | sigma_x_only_x0p100_y0p100_t0p080 | 0.417 | 9.662 | 0.323 | 68.459 |
| 10 | sigma_t_only_x0p100_y0p100_t0p040 | 0.417 | 9.840 | 0.313 | 69.165 |

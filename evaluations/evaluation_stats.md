# Distance evaluation stats

## Metrics (per bag, per method)

- **count**: Number of lidar scans in the bag for which this method produced a valid distance (meters).
- **mean**: Mean of those distances (average wall distance over the run).
- **std**: Standard deviation of the distances (spread; higher = more variation/oscillation).
- **min** / **max**: Minimum and maximum distance observed.
- **mean_abs_error**: Average absolute error (meters) vs the desired wall distance of **0.6 m**; i.e. mean of |distance − 0.6| over scans. Lower = closer to target.

Each **method** is a different way of estimating "distance to the wall" from the same lidar scan:
- **muktha**, **tissany**, **ansisg**, **jeryl**: The four branch algorithms (angle-filtered line fit, half-scan linregress, IEPF segments + raycast, polyfit on side filter).
- **median_of_4**: For each scan, the median of the four methods' distances; then we report count/mean/std/min/max over those per-scan medians. Use this to compare a single aggregate estimate vs individual methods.

**Test case structure:** Each test case has 3 runs (e.g. straight_inward_muktha_left_1, _2, _3). There are 3 runs per (runner, side): 3 left + 3 right per runner. The **per test case (3 runs)** tables pool all scans from the 3 bags in that test case and report one row per (test_case, method).

**tissany_left:** Bags in this test case are truncated to the average scan count of the other two left runs (_2 and _3) so the long run (_1) does not dominate the pooled stats.

---

## Method: `muktha`

### Per bag (individual runs)

| bag | side | count | mean | std | min | max | avg error |
|-----|------|-------|------|-----|-----|-----|----------|
| straight_inward_ansis_left_1 | left | 929 | 0.7918 | 0.2787 | 0.5883 | 1.3765 | 0.1920 |
| straight_inward_ansis_left_2 | left | 907 | 0.7745 | 0.2642 | 0.6098 | 1.3734 | 0.1745 |
| straight_inward_ansis_left_3 | left | 916 | 0.7792 | 0.2714 | 0.5917 | 1.3717 | 0.1795 |
| straight_inward_ansis_right_1 | right | 371 | 0.8191 | 0.6261 | 0.1479 | 1.5715 | 0.6147 |
| straight_inward_ansis_right_2 | right | 549 | 0.9713 | 0.4315 | 0.2395 | 1.5473 | 0.4451 |
| straight_inward_ansis_right_3 | right | 572 | 0.9875 | 0.4179 | 0.2387 | 1.5406 | 0.4453 |
| straight_inward_jeryl_left_1 | left | 973 | 0.7595 | 0.2978 | 0.5464 | 1.3727 | 0.1660 |
| straight_inward_jeryl_left_2 | left | 1000 | 0.7990 | 0.3264 | 0.5652 | 1.3760 | 0.2054 |
| straight_inward_jeryl_left_3 | left | 1038 | 0.7922 | 0.3209 | 0.5527 | 1.3702 | 0.1985 |
| straight_inward_jeryl_right_1 | right | 652 | 1.1538 | 0.5498 | 0.4998 | 2.2673 | 0.5897 |
| straight_inward_jeryl_right_2 | right | 561 | 1.0859 | 0.5371 | 0.4989 | 2.1647 | 0.5140 |
| straight_inward_jeryl_right_3 | right | 624 | 1.1530 | 0.5492 | 0.5068 | 2.2454 | 0.5833 |
| straight_inward_muktha_left_1 | left | 864 | 0.8948 | 0.2354 | 0.7000 | 1.3738 | 0.2948 |
| straight_inward_muktha_left_2 | left | 896 | 0.9157 | 0.2465 | 0.7042 | 1.3787 | 0.3157 |
| straight_inward_muktha_left_3 | left | 926 | 0.9371 | 0.2595 | 0.7003 | 1.3752 | 0.3371 |
| straight_inward_muktha_right_1 | right | 571 | 1.1261 | 0.3344 | 0.6079 | 1.5701 | 0.5261 |
| straight_inward_muktha_right_2 | right | 612 | 1.0951 | 0.3377 | 0.6337 | 1.5461 | 0.4951 |
| straight_inward_muktha_right_3 | right | 497 | 1.0760 | 0.3313 | 0.6163 | 1.5564 | 0.4760 |
| straight_inward_tissany_left_1 | left | 946 | 0.8892 | 0.2692 | 0.6614 | 1.3766 | 0.2892 |
| straight_inward_tissany_left_2 | left | 946 | 0.9240 | 0.2909 | 0.6575 | 1.3785 | 0.3240 |
| straight_inward_tissany_left_3 | left | 917 | 0.8415 | 0.2233 | 0.6539 | 1.3736 | 0.2415 |
| straight_inward_tissany_right_1 | right | 527 | 1.0827 | 0.3499 | 0.6471 | 1.5892 | 0.4827 |
| straight_inward_tissany_right_2 | right | 460 | 1.0465 | 0.3480 | 0.6521 | 1.5653 | 0.4465 |
| straight_inward_tissany_right_3 | right | 544 | 1.1040 | 0.3648 | 0.6499 | 1.5665 | 0.5040 |

### Per test case (3 runs pooled)

| test case | side | count | mean | std | min | max | avg error |
|------------|------|-------|------|-----|-----|-----|----------|
| straight_inward_ansis_left (3 runs) | left | 2752 | 0.7819 | 0.2716 | 0.5883 | 1.3765 | 0.1821 |
| straight_inward_ansis_right (3 runs) | right | 1492 | 0.9397 | 0.4877 | 0.1479 | 1.5715 | 0.4874 |
| straight_inward_jeryl_left (3 runs) | left | 3011 | 0.7839 | 0.3160 | 0.5464 | 1.3760 | 0.1903 |
| straight_inward_jeryl_right (3 runs) | right | 1837 | 1.1328 | 0.5466 | 0.4989 | 2.2673 | 0.5644 |
| straight_inward_muktha_left (3 runs) | left | 2686 | 0.9163 | 0.2482 | 0.7000 | 1.3787 | 0.3163 |
| straight_inward_muktha_right (3 runs) | right | 1680 | 1.1000 | 0.3353 | 0.6079 | 1.5701 | 0.5000 |
| straight_inward_tissany_left (3 runs) | left | 2809 | 0.8853 | 0.2651 | 0.6539 | 1.3785 | 0.2853 |
| straight_inward_tissany_right (3 runs) | right | 1531 | 1.0794 | 0.3555 | 0.6471 | 1.5892 | 0.4794 |

## Method: `tissany`

### Per bag (individual runs)

| bag | side | count | mean | std | min | max | avg error |
|-----|------|-------|------|-----|-----|-----|----------|
| straight_inward_ansis_left_1 | left | 929 | 0.7056 | 0.2730 | 0.4559 | 1.2665 | 0.1826 |
| straight_inward_ansis_left_2 | left | 907 | 0.6840 | 0.2658 | 0.4694 | 1.3058 | 0.1707 |
| straight_inward_ansis_left_3 | left | 916 | 0.7049 | 0.2780 | 0.4680 | 1.3278 | 0.1749 |
| straight_inward_ansis_right_1 | right | 371 | 1.6798 | 1.5978 | 0.1343 | 5.5742 | 1.4709 |
| straight_inward_ansis_right_2 | right | 549 | 1.5474 | 1.1422 | 0.2469 | 5.5929 | 1.0258 |
| straight_inward_ansis_right_3 | right | 572 | 1.5488 | 1.0995 | 0.2413 | 5.0508 | 1.0182 |
| straight_inward_jeryl_left_1 | left | 973 | 0.6798 | 0.2936 | 0.4495 | 1.3645 | 0.1906 |
| straight_inward_jeryl_left_2 | left | 1000 | 0.7105 | 0.3138 | 0.4426 | 1.2608 | 0.2213 |
| straight_inward_jeryl_left_3 | left | 1038 | 0.7141 | 0.3220 | 0.4369 | 1.3113 | 0.2210 |
| straight_inward_jeryl_right_1 | right | 652 | 2.5531 | 2.8967 | 0.3505 | 13.3041 | 2.0351 |
| straight_inward_jeryl_right_2 | right | 561 | 2.4465 | 2.8327 | 0.3264 | 13.3572 | 1.9239 |
| straight_inward_jeryl_right_3 | right | 624 | 2.7033 | 2.8971 | 0.3592 | 13.5185 | 2.1795 |
| straight_inward_muktha_left_1 | left | 864 | 0.8145 | 0.2607 | 0.5307 | 1.6582 | 0.2290 |
| straight_inward_muktha_left_2 | left | 896 | 0.8297 | 0.2596 | 0.5340 | 1.5069 | 0.2415 |
| straight_inward_muktha_left_3 | left | 926 | 0.8555 | 0.2803 | 0.5343 | 1.7262 | 0.2666 |
| straight_inward_muktha_right_1 | right | 571 | 1.6083 | 0.9821 | 0.4659 | 5.7666 | 1.0203 |
| straight_inward_muktha_right_2 | right | 612 | 1.7126 | 1.1807 | 0.4815 | 6.0312 | 1.1217 |
| straight_inward_muktha_right_3 | right | 497 | 1.6266 | 1.1322 | 0.4743 | 5.5630 | 1.0380 |
| straight_inward_tissany_left_1 | left | 946 | 0.8228 | 0.2920 | 0.4871 | 1.8527 | 0.2475 |
| straight_inward_tissany_left_2 | left | 946 | 0.8559 | 0.3076 | 0.4940 | 1.7794 | 0.2807 |
| straight_inward_tissany_left_3 | left | 917 | 0.7931 | 0.2806 | 0.4898 | 1.9451 | 0.2186 |
| straight_inward_tissany_right_1 | right | 527 | 1.7517 | 1.2581 | 0.4927 | 5.8478 | 1.1693 |
| straight_inward_tissany_right_2 | right | 460 | 1.6184 | 1.2285 | 0.4955 | 5.5830 | 1.0380 |
| straight_inward_tissany_right_3 | right | 544 | 2.6054 | 2.2142 | 0.4920 | 8.5078 | 2.0220 |

### Per test case (3 runs pooled)

| test case | side | count | mean | std | min | max | avg error |
|------------|------|-------|------|-----|-----|-----|----------|
| straight_inward_ansis_left (3 runs) | left | 2752 | 0.6982 | 0.2725 | 0.4559 | 1.3278 | 0.1761 |
| straight_inward_ansis_right (3 runs) | right | 1492 | 1.5808 | 1.2576 | 0.1343 | 5.5929 | 1.1335 |
| straight_inward_jeryl_left (3 runs) | left | 3011 | 0.7018 | 0.3107 | 0.4369 | 1.3645 | 0.2113 |
| straight_inward_jeryl_right (3 runs) | right | 1837 | 2.5716 | 2.8793 | 0.3264 | 13.5185 | 2.0502 |
| straight_inward_muktha_left (3 runs) | left | 2686 | 0.8337 | 0.2678 | 0.5307 | 1.7262 | 0.2461 |
| straight_inward_muktha_right (3 runs) | right | 1680 | 1.6517 | 1.1032 | 0.4659 | 6.0312 | 1.0625 |
| straight_inward_tissany_left (3 runs) | left | 2809 | 0.8242 | 0.2948 | 0.4871 | 1.9451 | 0.2492 |
| straight_inward_tissany_right (3 runs) | right | 1531 | 2.0150 | 1.7133 | 0.4920 | 8.5078 | 1.4328 |

## Method: `ansisg`

### Per bag (individual runs)

| bag | side | count | mean | std | min | max | avg error |
|-----|------|-------|------|-----|-----|-----|----------|
| straight_inward_ansis_left_1 | left | 929 | 0.7853 | 0.2837 | 0.5940 | 1.3782 | 0.1858 |
| straight_inward_ansis_left_2 | left | 907 | 0.7683 | 0.2689 | 0.6102 | 1.3757 | 0.1683 |
| straight_inward_ansis_left_3 | left | 916 | 0.7736 | 0.2753 | 0.5786 | 1.3731 | 0.1772 |
| straight_inward_ansis_right_1 | right | 371 | 1.0946 | 0.9323 | 0.1444 | 2.2224 | 0.8909 |
| straight_inward_ansis_right_2 | right | 549 | 1.1871 | 0.6768 | 0.2501 | 2.0953 | 0.6585 |
| straight_inward_ansis_right_3 | right | 572 | 1.2284 | 0.6814 | 0.2475 | 2.0894 | 0.6833 |
| straight_inward_jeryl_left_1 | left | 973 | 0.7528 | 0.3034 | 0.3953 | 1.3738 | 0.1682 |
| straight_inward_jeryl_left_2 | left | 1000 | 0.7920 | 0.3330 | 0.0755 | 1.3765 | 0.2078 |
| straight_inward_jeryl_left_3 | left | 1038 | 0.7853 | 0.3272 | 0.3330 | 1.3722 | 0.2001 |
| straight_inward_jeryl_right_1 | right | 652 | 1.4046 | 0.7548 | 0.4967 | 2.3344 | 0.8431 |
| straight_inward_jeryl_right_2 | right | 561 | 1.2973 | 0.7587 | 0.5004 | 2.2455 | 0.7299 |
| straight_inward_jeryl_right_3 | right | 624 | 1.4135 | 0.7791 | 0.5053 | 2.3173 | 0.8466 |
| straight_inward_muktha_left_1 | left | 864 | 0.8899 | 0.2402 | 0.6605 | 1.3759 | 0.2899 |
| straight_inward_muktha_left_2 | left | 896 | 0.9115 | 0.2510 | 0.6982 | 1.3792 | 0.3115 |
| straight_inward_muktha_left_3 | left | 926 | 0.9332 | 0.2638 | 0.6932 | 1.3758 | 0.3332 |
| straight_inward_muktha_right_1 | right | 571 | 1.4469 | 0.6743 | 0.6036 | 2.2011 | 0.8469 |
| straight_inward_muktha_right_2 | right | 612 | 1.3428 | 0.6326 | 0.6184 | 2.1222 | 0.7428 |
| straight_inward_muktha_right_3 | right | 497 | 1.3168 | 0.6298 | 0.6112 | 2.1457 | 0.7168 |
| straight_inward_tissany_left_1 | left | 946 | 0.8842 | 0.2740 | 0.6614 | 1.3782 | 0.2842 |
| straight_inward_tissany_left_2 | left | 946 | 0.9203 | 0.2958 | 0.6557 | 1.3801 | 0.3203 |
| straight_inward_tissany_left_3 | left | 917 | 0.8455 | 0.2467 | 0.6568 | 1.3758 | 0.2455 |
| straight_inward_tissany_right_1 | right | 527 | 1.4060 | 0.7319 | 0.6466 | 2.3108 | 0.8060 |
| straight_inward_tissany_right_2 | right | 460 | 1.2702 | 0.6536 | 0.6119 | 2.1929 | 0.6702 |
| straight_inward_tissany_right_3 | right | 544 | 1.3940 | 0.6982 | 0.6486 | 2.2458 | 0.7940 |

### Per test case (3 runs pooled)

| test case | side | count | mean | std | min | max | avg error |
|------------|------|-------|------|-----|-----|-----|----------|
| straight_inward_ansis_left (3 runs) | left | 2752 | 0.7758 | 0.2762 | 0.5786 | 1.3782 | 0.1772 |
| straight_inward_ansis_right (3 runs) | right | 1492 | 1.1799 | 0.7519 | 0.1444 | 2.2224 | 0.7258 |
| straight_inward_jeryl_left (3 runs) | left | 3011 | 0.7770 | 0.3221 | 0.0755 | 1.3765 | 0.1923 |
| straight_inward_jeryl_right (3 runs) | right | 1837 | 1.3749 | 0.7661 | 0.4967 | 2.3344 | 0.8097 |
| straight_inward_muktha_left (3 runs) | left | 2686 | 0.9121 | 0.2527 | 0.6605 | 1.3792 | 0.3121 |
| straight_inward_muktha_right (3 runs) | right | 1680 | 1.3705 | 0.6487 | 0.6036 | 2.2011 | 0.7705 |
| straight_inward_tissany_left (3 runs) | left | 2809 | 0.8837 | 0.2749 | 0.6557 | 1.3801 | 0.2837 |
| straight_inward_tissany_right (3 runs) | right | 1531 | 1.3609 | 0.6997 | 0.6119 | 2.3108 | 0.7609 |

## Method: `jeryl`

### Per bag (individual runs)

| bag | side | count | mean | std | min | max | avg error |
|-----|------|-------|------|-----|-----|-----|----------|
| straight_inward_ansis_left_1 | left | 929 | 0.8045 | 0.2762 | 0.5938 | 1.3780 | 0.2048 |
| straight_inward_ansis_left_2 | left | 907 | 0.7888 | 0.2608 | 0.6134 | 1.3748 | 0.1888 |
| straight_inward_ansis_left_3 | left | 916 | 0.7942 | 0.2680 | 0.5880 | 1.3728 | 0.1952 |
| straight_inward_ansis_right_1 | right | 371 | 1.1335 | 0.9846 | 0.1440 | 2.4277 | 0.9333 |
| straight_inward_ansis_right_2 | right | 549 | 1.3016 | 0.8115 | 0.2501 | 2.4223 | 0.7742 |
| straight_inward_ansis_right_3 | right | 572 | 1.3529 | 0.8260 | 0.2475 | 2.4202 | 0.8080 |
| straight_inward_jeryl_left_1 | left | 973 | 0.7743 | 0.2928 | 0.5778 | 1.3737 | 0.1778 |
| straight_inward_jeryl_left_2 | left | 1000 | 0.8112 | 0.3214 | 0.5690 | 1.3758 | 0.2156 |
| straight_inward_jeryl_left_3 | left | 1038 | 0.8063 | 0.3149 | 0.5750 | 1.3713 | 0.2100 |
| straight_inward_jeryl_right_1 | right | 652 | 1.5480 | 0.8654 | 0.5334 | 2.8936 | 0.9678 |
| straight_inward_jeryl_right_2 | right | 561 | 1.3747 | 0.8058 | 0.5004 | 2.8080 | 0.7901 |
| straight_inward_jeryl_right_3 | right | 624 | 1.4950 | 0.8276 | 0.5081 | 2.8874 | 0.9065 |
| straight_inward_muktha_left_1 | left | 864 | 0.9122 | 0.2260 | 0.7231 | 1.3752 | 0.3122 |
| straight_inward_muktha_left_2 | left | 896 | 0.9324 | 0.2369 | 0.7279 | 1.3791 | 0.3324 |
| straight_inward_muktha_left_3 | left | 926 | 0.9526 | 0.2494 | 0.7271 | 1.3754 | 0.3526 |
| straight_inward_muktha_right_1 | right | 571 | 1.5503 | 0.7550 | 0.6294 | 2.4371 | 0.9503 |
| straight_inward_muktha_right_2 | right | 612 | 1.4778 | 0.7387 | 0.6553 | 2.4205 | 0.8778 |
| straight_inward_muktha_right_3 | right | 497 | 1.4340 | 0.7395 | 0.6370 | 2.4376 | 0.8340 |
| straight_inward_tissany_left_1 | left | 946 | 0.9075 | 0.2595 | 0.6688 | 1.3780 | 0.3075 |
| straight_inward_tissany_left_2 | left | 946 | 0.9391 | 0.2818 | 0.6658 | 1.3796 | 0.3391 |
| straight_inward_tissany_left_3 | left | 917 | 0.8712 | 0.2325 | 0.6652 | 1.3757 | 0.2712 |
| straight_inward_tissany_right_1 | right | 527 | 1.4723 | 0.7645 | 0.6660 | 2.4419 | 0.8723 |
| straight_inward_tissany_right_2 | right | 460 | 1.3676 | 0.7363 | 0.6682 | 2.4389 | 0.7676 |
| straight_inward_tissany_right_3 | right | 544 | 1.3285 | 0.5867 | 0.6679 | 2.4164 | 0.7285 |

### Per test case (3 runs pooled)

| test case | side | count | mean | std | min | max | avg error |
|------------|------|-------|------|-----|-----|-----|----------|
| straight_inward_ansis_left (3 runs) | left | 2752 | 0.7959 | 0.2685 | 0.5880 | 1.3780 | 0.1964 |
| straight_inward_ansis_right (3 runs) | right | 1492 | 1.2795 | 0.8675 | 0.1440 | 2.4277 | 0.8267 |
| straight_inward_jeryl_left (3 runs) | left | 3011 | 0.7976 | 0.3106 | 0.5690 | 1.3758 | 0.2015 |
| straight_inward_jeryl_right (3 runs) | right | 1837 | 1.4771 | 0.8378 | 0.5004 | 2.8936 | 0.8927 |
| straight_inward_muktha_left (3 runs) | left | 2686 | 0.9329 | 0.2385 | 0.7231 | 1.3791 | 0.3329 |
| straight_inward_muktha_right (3 runs) | right | 1680 | 1.4895 | 0.7460 | 0.6294 | 2.4376 | 0.8895 |
| straight_inward_tissany_left (3 runs) | left | 2809 | 0.9063 | 0.2605 | 0.6652 | 1.3796 | 0.3063 |
| straight_inward_tissany_right (3 runs) | right | 1531 | 1.3897 | 0.7002 | 0.6660 | 2.4419 | 0.7897 |

## Method: `median_of_4`

### Per bag (individual runs)

| bag | side | count | mean | std | min | max | avg error |
|-----|------|-------|------|-----|-----|-----|----------|
| straight_inward_ansis_left_1 | left | 929 | 0.7868 | 0.2819 | 0.5910 | 1.3768 | 0.1871 |
| straight_inward_ansis_left_2 | left | 907 | 0.7692 | 0.2673 | 0.6118 | 1.3738 | 0.1692 |
| straight_inward_ansis_left_3 | left | 916 | 0.7747 | 0.2742 | 0.5868 | 1.3717 | 0.1769 |
| straight_inward_ansis_right_1 | right | 371 | 1.1101 | 0.9534 | 0.1442 | 2.3011 | 0.9077 |
| straight_inward_ansis_right_2 | right | 549 | 1.2261 | 0.7377 | 0.2501 | 2.2549 | 0.6987 |
| straight_inward_ansis_right_3 | right | 572 | 1.2685 | 0.7424 | 0.2475 | 2.2504 | 0.7249 |
| straight_inward_jeryl_left_1 | left | 973 | 0.7547 | 0.3013 | 0.5149 | 1.3731 | 0.1662 |
| straight_inward_jeryl_left_2 | left | 1000 | 0.7939 | 0.3302 | 0.5273 | 1.3754 | 0.2054 |
| straight_inward_jeryl_left_3 | left | 1038 | 0.7871 | 0.3247 | 0.5096 | 1.3706 | 0.1980 |
| straight_inward_jeryl_right_1 | right | 652 | 1.4630 | 0.8125 | 0.4982 | 2.6078 | 0.9005 |
| straight_inward_jeryl_right_2 | right | 561 | 1.3251 | 0.7901 | 0.4998 | 2.5210 | 0.7562 |
| straight_inward_jeryl_right_3 | right | 624 | 1.4455 | 0.8103 | 0.5078 | 2.6022 | 0.8776 |
| straight_inward_muktha_left_1 | left | 864 | 0.8915 | 0.2382 | 0.6971 | 1.3744 | 0.2915 |
| straight_inward_muktha_left_2 | left | 896 | 0.9127 | 0.2494 | 0.7014 | 1.3784 | 0.3127 |
| straight_inward_muktha_left_3 | left | 926 | 0.9343 | 0.2622 | 0.6970 | 1.3752 | 0.3343 |
| straight_inward_muktha_right_1 | right | 571 | 1.4612 | 0.6889 | 0.6062 | 2.3138 | 0.8612 |
| straight_inward_muktha_right_2 | right | 612 | 1.3898 | 0.6870 | 0.6311 | 2.2701 | 0.7898 |
| straight_inward_muktha_right_3 | right | 497 | 1.3564 | 0.6796 | 0.6138 | 2.2907 | 0.7564 |
| straight_inward_tissany_left_1 | left | 946 | 0.8857 | 0.2723 | 0.6648 | 1.3773 | 0.2857 |
| straight_inward_tissany_left_2 | left | 946 | 0.9213 | 0.2940 | 0.6606 | 1.3790 | 0.3213 |
| straight_inward_tissany_left_3 | left | 917 | 0.8432 | 0.2369 | 0.6602 | 1.3742 | 0.2432 |
| straight_inward_tissany_right_1 | right | 527 | 1.4243 | 0.7489 | 0.6484 | 2.3686 | 0.8243 |
| straight_inward_tissany_right_2 | right | 460 | 1.3053 | 0.6952 | 0.6523 | 2.3152 | 0.7053 |
| straight_inward_tissany_right_3 | right | 544 | 1.3507 | 0.6446 | 0.6503 | 2.2794 | 0.7507 |

### Per test case (3 runs pooled)

| test case | side | count | mean | std | min | max | avg error |
|------------|------|-------|------|-----|-----|-----|----------|
| straight_inward_ansis_left (3 runs) | left | 2752 | 0.7770 | 0.2747 | 0.5868 | 1.3768 | 0.1778 |
| straight_inward_ansis_right (3 runs) | right | 1492 | 1.2135 | 0.8009 | 0.1442 | 2.3011 | 0.7607 |
| straight_inward_jeryl_left (3 runs) | left | 3011 | 0.7789 | 0.3197 | 0.5096 | 1.3754 | 0.1902 |
| straight_inward_jeryl_right (3 runs) | right | 1837 | 1.4149 | 0.8072 | 0.4982 | 2.6078 | 0.8486 |
| straight_inward_muktha_left (3 runs) | left | 2686 | 0.9133 | 0.2510 | 0.6970 | 1.3784 | 0.3133 |
| straight_inward_muktha_right (3 runs) | right | 1680 | 1.4042 | 0.6868 | 0.6062 | 2.3138 | 0.8042 |
| straight_inward_tissany_left (3 runs) | left | 2809 | 0.8838 | 0.2709 | 0.6602 | 1.3790 | 0.2838 |
| straight_inward_tissany_right (3 runs) | right | 1531 | 1.3624 | 0.6987 | 0.6484 | 2.3686 | 0.7624 |


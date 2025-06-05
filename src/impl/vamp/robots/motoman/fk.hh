
#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

namespace vamp::robots::motoman {

  using Configuration = FloatVector<16>;
  template <std::size_t block_width> using ConfigurationBlock = FloatVector<block_width, 16>;

  alignas(Configuration::S::Alignment) constexpr std::array<float, 16> s_m_a{5.93411945678, 5.93411945678, 6.28318530718, 3.83972435438, 5.93411945678, 4.71238898038, 6.28318530718, 3.83972435438, 6.28318530718, 6.28318530718, 3.83972435438, 5.93411945678, 4.71238898038, 6.28318530718, 3.83972435438, 6.28318530718};
  alignas(Configuration::S::Alignment) constexpr std::array<float, 16> s_a_a{-2.96705972839, -2.96705972839, -3.14159265359, -1.91986217719, -2.96705972839, -2.35619449019, -3.14159265359, -1.91986217719, -3.14159265359, -3.14159265359, -1.91986217719, -2.96705972839, -2.35619449019, -3.14159265359, -1.91986217719, -3.14159265359};
  static const Configuration s_m(s_m_a);
  static const Configuration s_a(s_a_a);inline void scale_configuration(Configuration& q) noexcept { q = q * s_m + s_a; }template <std::size_t block_width> inline void scale_configuration_block(ConfigurationBlock<block_width>& q) noexcept {q[0] = -2.96705972839 + (q[0] * 5.93411945678);
  q[1] = -2.96705972839 + (q[1] * 5.93411945678);
  q[2] = -3.14159265359 + (q[2] * 6.28318530718);
  q[3] = -1.91986217719 + (q[3] * 3.83972435438);
  q[4] = -2.96705972839 + (q[4] * 5.93411945678);
  q[5] = -2.35619449019 + (q[5] * 4.71238898038);
  q[6] = -3.14159265359 + (q[6] * 6.28318530718);
  q[7] = -1.91986217719 + (q[7] * 3.83972435438);
  q[8] = -3.14159265359 + (q[8] * 6.28318530718);
  q[9] = -3.14159265359 + (q[9] * 6.28318530718);
  q[10] = -1.91986217719 + (q[10] * 3.83972435438);
  q[11] = -2.96705972839 + (q[11] * 5.93411945678);
  q[12] = -2.35619449019 + (q[12] * 4.71238898038);
  q[13] = -3.14159265359 + (q[13] * 6.28318530718);
  q[14] = -1.91986217719 + (q[14] * 3.83972435438);
  q[15] = -3.14159265359 + (q[15] * 6.28318530718);}

  alignas(Configuration::S::Alignment) constexpr std::array<float, 16> d_m_a{0.16851699856790964, 0.16851699856790964, 0.15915494309188485, 0.26043536142361184, 0.16851699856790964, 0.21220659078940496, 0.15915494309188485, 0.26043536142361184, 0.15915494309188485, 0.15915494309188485, 0.26043536142361184, 0.16851699856790964, 0.21220659078940496, 0.15915494309188485, 0.26043536142361184, 0.15915494309188485};
  alignas(Configuration::S::Alignment) constexpr std::array<float, 16> d_a_a{-2.96705972839, -2.96705972839, -3.14159265359, -1.91986217719, -2.96705972839, -2.35619449019, -3.14159265359, -1.91986217719, -3.14159265359, -3.14159265359, -1.91986217719, -2.96705972839, -2.35619449019, -3.14159265359, -1.91986217719, -3.14159265359};
  static const Configuration d_m(d_m_a);
  static const Configuration d_a(d_a_a);inline void descale_configuration(Configuration& q) noexcept { q = (q - d_a) * d_m; }template <std::size_t block_width> inline void descale_configuration_block(ConfigurationBlock<block_width>& q) noexcept {q[0] = 0.16851699856790964 * (q[0] - -2.96705972839);
  q[1] = 0.16851699856790964 * (q[1] - -2.96705972839);
  q[2] = 0.15915494309188485 * (q[2] - -3.14159265359);
  q[3] = 0.26043536142361184 * (q[3] - -1.91986217719);
  q[4] = 0.16851699856790964 * (q[4] - -2.96705972839);
  q[5] = 0.21220659078940496 * (q[5] - -2.35619449019);
  q[6] = 0.15915494309188485 * (q[6] - -3.14159265359);
  q[7] = 0.26043536142361184 * (q[7] - -1.91986217719);
  q[8] = 0.15915494309188485 * (q[8] - -3.14159265359);
  q[9] = 0.15915494309188485 * (q[9] - -3.14159265359);
  q[10] = 0.26043536142361184 * (q[10] - -1.91986217719);
  q[11] = 0.16851699856790964 * (q[11] - -2.96705972839);
  q[12] = 0.21220659078940496 * (q[12] - -2.35619449019);
  q[13] = 0.15915494309188485 * (q[13] - -3.14159265359);
  q[14] = 0.26043536142361184 * (q[14] - -1.91986217719);
  q[15] = 0.15915494309188485 * (q[15] - -3.14159265359);}

  constexpr auto space_measure = 241906172900177.47;

    constexpr auto n_spheres = 35;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, 35> x;
        FloatVector<rake, 35> y;
        FloatVector<rake, 35> z;
        FloatVector<rake, 35> r;
    };

  template <std::size_t rake>
  inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept {
    return;
  }

  template <std::size_t rake>
  inline void interleaved_sphere_fk(const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept  {
  // if(/*torso_base_link*/ sphere_environment_in_collision(environment, -0.0552689, -1e-06, 0.2044429, 0.344575)){ if(sphere_environment_in_collision(environment, -0.0552689, -1e-06, 0.2044429, 0.344575)){ return false; }
  // if(sphere_environment_in_collision(environment, -0.0262329, -5e-06, 0.6608069, 0.273417)){ return false; }
  // if(sphere_environment_in_collision(environment, -0.1339749, 5e-06, 0.1743459, 0.325854)){ return false; } } // (0, 0)
  auto INPUT_0 = q[0];
  auto DIV_17 = INPUT_0 * 0.5;
  auto SIN_18 = DIV_17.sin();
  auto COS_24 = DIV_17.cos();
  auto MUL_2387 = COS_24 * SIN_18;
  auto MUL_2406 = MUL_2387 * 2.0;
  auto MUL_2437 = MUL_2406 * 5e-06;
  auto MUL_2386 = SIN_18 * SIN_18;
  auto MUL_2396 = MUL_2386 * 2.0;
  auto SUB_2399 = 1.0 - MUL_2396;
  auto MUL_2426 = SUB_2399 * 0.0262329;
  auto SUB_2453 = MUL_2437 - MUL_2426;
  auto MUL_2440 = SUB_2399 * 5e-06;
  auto MUL_2430 = MUL_2406 * 0.0262329;
  auto ADD_2454 = MUL_2430 + MUL_2440;
  auto NEGATE_2455 = -ADD_2454;
  auto MUL_2461 = SUB_2399 * 0.0884418;
  auto MUL_2468 = MUL_2406 * 1.4e-05;
  auto ADD_2484 = MUL_2461 + MUL_2468;
  auto MUL_2463 = MUL_2406 * 0.0884418;
  auto MUL_2471 = SUB_2399 * 1.4e-05;
  auto SUB_2485 = MUL_2463 - MUL_2471;
  auto MUL_2491 = SUB_2399 * 0.0276068;
  auto MUL_2497 = MUL_2406 * 1e-06;
  auto SUB_2511 = MUL_2491 - MUL_2497;
  auto MUL_2493 = MUL_2406 * 0.0276068;
  auto MUL_2500 = SUB_2399 * 1e-06;
  auto ADD_2512 = MUL_2493 + MUL_2500;
  if(/*torso_link_b1*/ sphere_environment_in_collision(environment, SUB_2453, NEGATE_2455, 0.6608069, 0.273417)){ if(sphere_environment_in_collision(environment, ADD_2484, SUB_2485, 1.186491, 0.186756)){ return false; }
  if(sphere_environment_in_collision(environment, SUB_2511, ADD_2512, 1.010525, 0.187648)){ return false; } } // (0, 28)
  auto MUL_115 = COS_24 * 0.7071068;
  auto MUL_113 = SIN_18 * 0.7071068;
  auto MUL_125 = SIN_18 * 0.265;
  auto MUL_136 = COS_24 * MUL_125;
  auto MUL_134 = SIN_18 * 0.1;
  auto MUL_139 = SIN_18 * MUL_134;
  auto ADD_141 = MUL_136 + MUL_139;
  auto MUL_144 = ADD_141 * 2.0;
  auto SUB_147 = 0.1 - MUL_144;
  auto INPUT_2 = q[2];
  auto DIV_168 = INPUT_2 * 0.5;
  auto SIN_169 = DIV_168.sin();
  auto COS_178 = DIV_168.cos();
  auto MUL_176 = SIN_169 * 1.0;
  auto MUL_191 = MUL_115 * MUL_176;
  auto MUL_197 = MUL_115 * COS_178;
  auto MUL_200 = MUL_113 * MUL_176;
  auto SUB_183 = MUL_197 - MUL_200;
  auto ADD_202 = MUL_197 + MUL_200;
  auto MUL_195 = MUL_113 * COS_178;
  auto SUB_196 = MUL_195 - MUL_191;
  auto ADD_189 = MUL_191 + MUL_195;
  auto MUL_2597 = ADD_202 * SUB_196;
  auto MUL_2598 = ADD_202 * ADD_189;
  auto MUL_2596 = SUB_196 * SUB_196;
  auto MUL_2602 = SUB_183 * SUB_196;
  auto ADD_2630 = MUL_2602 + MUL_2598;
  auto MUL_2632 = ADD_2630 * 2.0;
  auto MUL_2707 = MUL_2632 * 0.0541677;
  auto MUL_2601 = SUB_183 * ADD_189;
  auto SUB_2617 = MUL_2601 - MUL_2597;
  auto MUL_2619 = SUB_2617 * 2.0;
  auto MUL_2696 = MUL_2619 * 0.0106934;
  auto MUL_2595 = ADD_189 * ADD_189;
  auto ADD_2604 = MUL_2595 + MUL_2596;
  auto MUL_2607 = ADD_2604 * 2.0;
  auto SUB_2610 = 1.0 - MUL_2607;
  auto MUL_2684 = SUB_2610 * 0.0008364;
  auto ADD_2712 = MUL_2684 + MUL_2696;
  auto SUB_2718 = MUL_2707 - ADD_2712;
  auto ADD_2721 = SUB_147 + SUB_2718;
  auto ADD_2611 = MUL_2601 + MUL_2597;
  auto MUL_2613 = ADD_2611 * 2.0;
  auto MUL_2688 = MUL_2613 * 0.0008364;
  auto MUL_2600 = ADD_202 * SUB_183;
  auto MUL_2599 = SUB_183 * SUB_183;
  auto ADD_2620 = MUL_2596 + MUL_2599;
  auto MUL_2623 = ADD_2620 * 2.0;
  auto SUB_2626 = 1.0 - MUL_2623;
  auto MUL_2700 = SUB_2626 * 0.0106934;
  auto ADD_2714 = MUL_2688 + MUL_2700;
  auto MUL_2603 = ADD_189 * SUB_196;
  auto SUB_2633 = MUL_2603 - MUL_2600;
  auto MUL_2635 = SUB_2633 * 2.0;
  auto MUL_2709 = MUL_2635 * 0.0541677;
  auto SUB_2719 = MUL_2709 - ADD_2714;
  auto MUL_149 = COS_24 * MUL_134;
  auto MUL_151 = SIN_18 * MUL_125;
  auto SUB_153 = MUL_149 - MUL_151;
  auto MUL_155 = SUB_153 * 2.0;
  auto ADD_157 = MUL_155 + 0.265;
  auto ADD_2722 = ADD_157 + SUB_2719;
  auto SUB_2614 = MUL_2602 - MUL_2598;
  auto ADD_2627 = MUL_2603 + MUL_2600;
  auto ADD_2636 = MUL_2595 + MUL_2599;
  auto MUL_2639 = ADD_2636 * 2.0;
  auto SUB_2642 = 1.0 - MUL_2639;
  auto MUL_2711 = SUB_2642 * 0.0541677;
  auto MUL_2629 = ADD_2627 * 2.0;
  auto MUL_2704 = MUL_2629 * 0.0106934;
  auto MUL_2616 = SUB_2614 * 2.0;
  auto MUL_2692 = MUL_2616 * 0.0008364;
  auto ADD_2716 = MUL_2692 + MUL_2704;
  auto SUB_2720 = MUL_2711 - ADD_2716;
  auto ADD_2723 = 1.2 + SUB_2720;
  if(/*arm_left_link_1_s*/ sphere_environment_in_collision(environment, ADD_2721, ADD_2722, ADD_2723, 0.153255)){ return false; } // (28, 103)
  auto MUL_236 = ADD_202 * 0.7071068;
  auto MUL_253 = SUB_196 * 0.7071068;
  auto MUL_239 = SUB_183 * 0.7071068;
  auto SUB_240 = MUL_239 - MUL_236;
  auto ADD_273 = MUL_236 + MUL_239;
  auto MUL_250 = ADD_189 * 0.7071068;
  auto SUB_255 = MUL_250 - MUL_253;
  auto ADD_266 = MUL_250 + MUL_253;
  auto INPUT_3 = q[3];
  auto DIV_310 = INPUT_3 * 0.5;
  auto SIN_311 = DIV_310.sin();
  auto COS_320 = DIV_310.cos();
  auto MUL_318 = SIN_311 * 1.0;
  auto MUL_333 = ADD_273 * MUL_318;
  auto MUL_339 = ADD_273 * COS_320;
  auto MUL_342 = ADD_266 * MUL_318;
  auto ADD_344 = MUL_339 + MUL_342;
  auto MUL_337 = ADD_266 * COS_320;
  auto SUB_338 = MUL_337 - MUL_333;
  auto MUL_2728 = ADD_344 * SUB_338;
  auto MUL_2727 = SUB_338 * SUB_338;
  auto MUL_328 = SUB_240 * MUL_318;
  auto MUL_322 = SUB_240 * COS_320;
  auto MUL_323 = SUB_255 * MUL_318;
  auto SUB_325 = MUL_322 - MUL_323;
  auto MUL_2733 = SUB_325 * SUB_338;
  auto MUL_330 = SUB_255 * COS_320;
  auto ADD_331 = MUL_328 + MUL_330;
  auto MUL_2729 = ADD_344 * ADD_331;
  auto ADD_2761 = MUL_2733 + MUL_2729;
  auto MUL_2763 = ADD_2761 * 2.0;
  auto MUL_2788 = MUL_2763 * 0.189475;
  auto MUL_2732 = SUB_325 * ADD_331;
  auto SUB_2748 = MUL_2732 - MUL_2728;
  auto MUL_2750 = SUB_2748 * 2.0;
  auto MUL_2781 = MUL_2750 * 1e-06;
  auto MUL_2726 = ADD_331 * ADD_331;
  auto ADD_2735 = MUL_2726 + MUL_2727;
  auto MUL_2738 = ADD_2735 * 2.0;
  auto SUB_2741 = 1.0 - MUL_2738;
  auto MUL_2775 = SUB_2741 * 0.0276068;
  auto ADD_2798 = MUL_2775 + MUL_2781;
  auto SUB_2801 = ADD_2798 - MUL_2788;
  auto ADD_2804 = SUB_147 + SUB_2801;
  auto ADD_2742 = MUL_2732 + MUL_2728;
  auto MUL_2744 = ADD_2742 * 2.0;
  auto MUL_2777 = MUL_2744 * 0.0276068;
  auto MUL_2731 = ADD_344 * SUB_325;
  auto MUL_2730 = SUB_325 * SUB_325;
  auto ADD_2751 = MUL_2727 + MUL_2730;
  auto MUL_2754 = ADD_2751 * 2.0;
  auto SUB_2757 = 1.0 - MUL_2754;
  auto MUL_2783 = SUB_2757 * 1e-06;
  auto ADD_2799 = MUL_2777 + MUL_2783;
  auto MUL_2734 = ADD_331 * SUB_338;
  auto SUB_2764 = MUL_2734 - MUL_2731;
  auto MUL_2766 = SUB_2764 * 2.0;
  auto MUL_2792 = MUL_2766 * 0.189475;
  auto SUB_2802 = ADD_2799 - MUL_2792;
  auto ADD_2805 = ADD_157 + SUB_2802;
  auto SUB_2745 = MUL_2733 - MUL_2729;
  auto ADD_2758 = MUL_2734 + MUL_2731;
  auto ADD_2767 = MUL_2726 + MUL_2730;
  auto MUL_2770 = ADD_2767 * 2.0;
  auto SUB_2773 = 1.0 - MUL_2770;
  auto MUL_2796 = SUB_2773 * 0.189475;
  auto MUL_2760 = ADD_2758 * 2.0;
  auto MUL_2785 = MUL_2760 * 1e-06;
  auto MUL_2747 = SUB_2745 * 2.0;
  auto MUL_2779 = MUL_2747 * 0.0276068;
  auto ADD_2800 = MUL_2779 + MUL_2785;
  auto SUB_2803 = ADD_2800 - MUL_2796;
  auto ADD_2806 = 1.2 + SUB_2803;
  auto MUL_2826 = MUL_2763 * 0.0468617;
  auto MUL_2820 = MUL_2750 * 0.001079;
  auto MUL_2809 = SUB_2741 * 0.000508;
  auto SUB_2831 = MUL_2820 - MUL_2809;
  auto ADD_2834 = SUB_2831 + MUL_2826;
  auto ADD_2837 = SUB_147 + ADD_2834;
  auto MUL_2828 = MUL_2766 * 0.0468617;
  auto MUL_2822 = SUB_2757 * 0.001079;
  auto MUL_2813 = MUL_2744 * 0.000508;
  auto SUB_2832 = MUL_2822 - MUL_2813;
  auto ADD_2835 = SUB_2832 + MUL_2828;
  auto ADD_2838 = ADD_157 + ADD_2835;
  auto MUL_2830 = SUB_2773 * 0.0468617;
  auto MUL_2824 = MUL_2760 * 0.001079;
  auto MUL_2817 = MUL_2747 * 0.000508;
  auto SUB_2833 = MUL_2824 - MUL_2817;
  auto ADD_2836 = SUB_2833 + MUL_2830;
  auto ADD_2839 = 1.2 + ADD_2836;
  auto MUL_2847 = MUL_2750 * 0.106206;
  auto MUL_2853 = MUL_2763 * 0.0173577;
  auto MUL_2841 = SUB_2741 * 0.002951;
  auto ADD_2858 = MUL_2841 + MUL_2847;
  auto ADD_2861 = ADD_2858 + MUL_2853;
  auto ADD_2864 = SUB_147 + ADD_2861;
  auto MUL_2855 = MUL_2766 * 0.0173577;
  auto MUL_2849 = SUB_2757 * 0.106206;
  auto MUL_2843 = MUL_2744 * 0.002951;
  auto ADD_2859 = MUL_2843 + MUL_2849;
  auto ADD_2862 = ADD_2859 + MUL_2855;
  auto ADD_2865 = ADD_157 + ADD_2862;
  auto MUL_2857 = SUB_2773 * 0.0173577;
  auto MUL_2851 = MUL_2760 * 0.106206;
  auto MUL_2845 = MUL_2747 * 0.002951;
  auto ADD_2860 = MUL_2845 + MUL_2851;
  auto ADD_2863 = ADD_2860 + MUL_2857;
  auto ADD_2866 = 1.2 + ADD_2863;
  if(/*arm_left_link_2_l*/ sphere_environment_in_collision(environment, ADD_2804, ADD_2805, ADD_2806, 0.187648)){ if(sphere_environment_in_collision(environment, ADD_2837, ADD_2838, ADD_2839, 0.095701)){ return false; }
  if(sphere_environment_in_collision(environment, ADD_2864, ADD_2865, ADD_2866, 0.119311)){ return false; } } // (103, 212)
  auto MUL_377 = ADD_344 * 0.7071068;
  auto MUL_392 = SUB_338 * 0.7071068;
  auto MUL_379 = SUB_325 * 0.7071068;
  auto SUB_408 = MUL_377 - MUL_379;
  auto ADD_380 = MUL_377 + MUL_379;
  auto MUL_390 = ADD_331 * 0.7071068;
  auto SUB_403 = MUL_392 - MUL_390;
  auto ADD_393 = MUL_390 + MUL_392;
  auto MUL_416 = SUB_338 * 0.36;
  auto MUL_426 = ADD_344 * MUL_416;
  auto MUL_419 = SUB_325 * 0.36;
  auto MUL_428 = ADD_331 * MUL_419;
  auto SUB_429 = MUL_428 - MUL_426;
  auto MUL_432 = SUB_429 * 2.0;
  auto ADD_455 = SUB_147 + MUL_432;
  auto INPUT_4 = q[4];
  auto DIV_459 = INPUT_4 * 0.5;
  auto SIN_460 = DIV_459.sin();
  auto COS_469 = DIV_459.cos();
  auto MUL_467 = SIN_460 * 1.0;
  auto MUL_482 = SUB_408 * MUL_467;
  auto MUL_488 = SUB_408 * COS_469;
  auto MUL_491 = SUB_403 * MUL_467;
  auto ADD_493 = MUL_488 + MUL_491;
  auto MUL_486 = SUB_403 * COS_469;
  auto SUB_487 = MUL_486 - MUL_482;
  auto MUL_2875 = ADD_493 * SUB_487;
  auto MUL_2874 = SUB_487 * SUB_487;
  auto MUL_477 = ADD_380 * MUL_467;
  auto MUL_471 = ADD_380 * COS_469;
  auto MUL_472 = ADD_393 * MUL_467;
  auto SUB_474 = MUL_471 - MUL_472;
  auto MUL_2880 = SUB_474 * SUB_487;
  auto MUL_479 = ADD_393 * COS_469;
  auto ADD_480 = MUL_477 + MUL_479;
  auto MUL_2876 = ADD_493 * ADD_480;
  auto ADD_2908 = MUL_2880 + MUL_2876;
  auto MUL_2910 = ADD_2908 * 2.0;
  auto MUL_2934 = MUL_2910 * 0.116149;
  auto MUL_2879 = SUB_474 * ADD_480;
  auto SUB_2895 = MUL_2879 - MUL_2875;
  auto MUL_2897 = SUB_2895 * 2.0;
  auto MUL_2928 = MUL_2897 * 0.0035861;
  auto MUL_2873 = ADD_480 * ADD_480;
  auto ADD_2882 = MUL_2873 + MUL_2874;
  auto MUL_2885 = ADD_2882 * 2.0;
  auto SUB_2888 = 1.0 - MUL_2885;
  auto MUL_2922 = SUB_2888 * 0.0006352;
  auto ADD_2939 = MUL_2922 + MUL_2928;
  auto ADD_2942 = ADD_2939 + MUL_2934;
  auto ADD_2945 = ADD_455 + ADD_2942;
  auto ADD_2889 = MUL_2879 + MUL_2875;
  auto MUL_2891 = ADD_2889 * 2.0;
  auto MUL_2924 = MUL_2891 * 0.0006352;
  auto MUL_2878 = ADD_493 * SUB_474;
  auto MUL_2877 = SUB_474 * SUB_474;
  auto ADD_2898 = MUL_2874 + MUL_2877;
  auto MUL_2901 = ADD_2898 * 2.0;
  auto SUB_2904 = 1.0 - MUL_2901;
  auto MUL_2930 = SUB_2904 * 0.0035861;
  auto ADD_2940 = MUL_2924 + MUL_2930;
  auto MUL_2881 = ADD_480 * SUB_487;
  auto SUB_2911 = MUL_2881 - MUL_2878;
  auto MUL_2913 = SUB_2911 * 2.0;
  auto MUL_2936 = MUL_2913 * 0.116149;
  auto ADD_2943 = ADD_2940 + MUL_2936;
  auto MUL_438 = SUB_338 * MUL_416;
  auto MUL_436 = SUB_325 * MUL_419;
  auto ADD_440 = MUL_436 + MUL_438;
  auto MUL_443 = ADD_440 * 2.0;
  auto SUB_446 = 0.36 - MUL_443;
  auto ADD_456 = ADD_157 + SUB_446;
  auto ADD_2946 = ADD_456 + ADD_2943;
  auto SUB_2892 = MUL_2880 - MUL_2876;
  auto ADD_2905 = MUL_2881 + MUL_2878;
  auto ADD_2914 = MUL_2873 + MUL_2877;
  auto MUL_2917 = ADD_2914 * 2.0;
  auto SUB_2920 = 1.0 - MUL_2917;
  auto MUL_2938 = SUB_2920 * 0.116149;
  auto MUL_2907 = ADD_2905 * 2.0;
  auto MUL_2932 = MUL_2907 * 0.0035861;
  auto MUL_2894 = SUB_2892 * 2.0;
  auto MUL_2926 = MUL_2894 * 0.0006352;
  auto ADD_2941 = MUL_2926 + MUL_2932;
  auto ADD_2944 = ADD_2941 + MUL_2938;
  auto MUL_447 = ADD_344 * MUL_419;
  auto MUL_449 = ADD_331 * MUL_416;
  auto ADD_451 = MUL_447 + MUL_449;
  auto MUL_453 = ADD_451 * 2.0;
  auto ADD_457 = 1.2 + MUL_453;
  auto ADD_2947 = ADD_457 + ADD_2944;
  if(/*arm_left_link_1_s vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_2945, ADD_2946, ADD_2947, 0.095372)){ return false; } // (212, 303)
  if(/*arm_left_link_3_e*/ sphere_environment_in_collision(environment, ADD_2945, ADD_2946, ADD_2947, 0.095372)){ return false; } // (303, 303)
  if(/*torso_base_link vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_2945, ADD_2946, ADD_2947, 0.095372)){ return false; } // (303, 303)
  if(/*torso_base_link vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_2945, ADD_2946, ADD_2947, 0.095372)){ return false; } // (303, 303)
  if(/*torso_base_link vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_2945, ADD_2946, ADD_2947, 0.095372)){ return false; } // (303, 303)
  if(/*torso_link_b1 vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_2945, ADD_2946, ADD_2947, 0.095372)){ return false; } // (303, 303)
  if(/*torso_link_b1 vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_2945, ADD_2946, ADD_2947, 0.095372)){ return false; } // (303, 303)
  auto MUL_2963 = MUL_2897 * 0.0184731;
  auto MUL_2969 = MUL_2910 * 0.011294;
  auto MUL_2952 = SUB_2888 * 0.0006778;
  auto SUB_2974 = MUL_2963 - MUL_2952;
  auto ADD_2977 = SUB_2974 + MUL_2969;
  auto ADD_2980 = ADD_455 + ADD_2977;
  auto MUL_2971 = MUL_2913 * 0.011294;
  auto MUL_2965 = SUB_2904 * 0.0184731;
  auto MUL_2956 = MUL_2891 * 0.0006778;
  auto SUB_2975 = MUL_2965 - MUL_2956;
  auto ADD_2978 = SUB_2975 + MUL_2971;
  auto ADD_2981 = ADD_456 + ADD_2978;
  auto MUL_2973 = SUB_2920 * 0.011294;
  auto MUL_2967 = MUL_2907 * 0.0184731;
  auto MUL_2960 = MUL_2894 * 0.0006778;
  auto SUB_2976 = MUL_2967 - MUL_2960;
  auto ADD_2979 = SUB_2976 + MUL_2973;
  auto ADD_2982 = ADD_457 + ADD_2979;
  if(/*arm_left_link_1_s vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_2980, ADD_2981, ADD_2982, 0.085433)){ return false; } // (303, 321)
  if(/*arm_left_link_3_e*/ sphere_environment_in_collision(environment, ADD_2980, ADD_2981, ADD_2982, 0.085433)){ return false; } // (321, 321)
  if(/*torso_base_link vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_2980, ADD_2981, ADD_2982, 0.085433)){ return false; } // (321, 321)
  if(/*torso_base_link vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_2980, ADD_2981, ADD_2982, 0.085433)){ return false; } // (321, 321)
  if(/*torso_base_link vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_2980, ADD_2981, ADD_2982, 0.085433)){ return false; } // (321, 321)
  if(/*torso_link_b1 vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_2980, ADD_2981, ADD_2982, 0.085433)){ return false; } // (321, 321)
  if(/*torso_link_b1 vs. arm_left_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_2980, ADD_2981, ADD_2982, 0.085433)){ return false; } // (321, 321)
  auto MUL_559 = ADD_493 * 0.7071068;
  auto MUL_556 = SUB_487 * 0.7071068;
  auto MUL_562 = SUB_474 * 0.7071068;
  auto SUB_531 = MUL_562 - MUL_559;
  auto ADD_564 = MUL_559 + MUL_562;
  auto MUL_553 = ADD_480 * 0.7071068;
  auto SUB_546 = MUL_553 - MUL_556;
  auto ADD_557 = MUL_553 + MUL_556;
  auto INPUT_5 = q[5];
  auto DIV_601 = INPUT_5 * 0.5;
  auto SIN_602 = DIV_601.sin();
  auto COS_608 = DIV_601.cos();
  auto MUL_620 = ADD_564 * SIN_602;
  auto MUL_625 = ADD_564 * COS_608;
  auto MUL_628 = ADD_557 * SIN_602;
  auto SUB_629 = MUL_625 - MUL_628;
  auto MUL_623 = ADD_557 * COS_608;
  auto ADD_624 = MUL_620 + MUL_623;
  auto MUL_2987 = SUB_629 * ADD_624;
  auto MUL_2986 = ADD_624 * ADD_624;
  auto MUL_615 = SUB_531 * SIN_602;
  auto MUL_610 = SUB_531 * COS_608;
  auto MUL_611 = SUB_546 * SIN_602;
  auto ADD_612 = MUL_610 + MUL_611;
  auto MUL_2992 = ADD_612 * ADD_624;
  auto MUL_617 = SUB_546 * COS_608;
  auto SUB_618 = MUL_617 - MUL_615;
  auto MUL_2988 = SUB_629 * SUB_618;
  auto ADD_3020 = MUL_2992 + MUL_2988;
  auto MUL_3022 = ADD_3020 * 2.0;
  auto MUL_3058 = MUL_3022 * 0.0541677;
  auto MUL_2985 = SUB_618 * SUB_618;
  auto ADD_2994 = MUL_2985 + MUL_2986;
  auto MUL_2997 = ADD_2994 * 2.0;
  auto SUB_3000 = 1.0 - MUL_2997;
  auto MUL_3035 = SUB_3000 * 0.0008364;
  auto MUL_2991 = ADD_612 * SUB_618;
  auto SUB_3007 = MUL_2991 - MUL_2987;
  auto MUL_3009 = SUB_3007 * 2.0;
  auto MUL_3047 = MUL_3009 * 0.0106934;
  auto ADD_3063 = MUL_3035 + MUL_3047;
  auto SUB_3069 = MUL_3058 - ADD_3063;
  auto ADD_3072 = ADD_455 + SUB_3069;
  auto ADD_3001 = MUL_2991 + MUL_2987;
  auto MUL_3003 = ADD_3001 * 2.0;
  auto MUL_3039 = MUL_3003 * 0.0008364;
  auto MUL_2990 = SUB_629 * ADD_612;
  auto MUL_2993 = SUB_618 * ADD_624;
  auto SUB_3023 = MUL_2993 - MUL_2990;
  auto MUL_3025 = SUB_3023 * 2.0;
  auto MUL_3060 = MUL_3025 * 0.0541677;
  auto MUL_2989 = ADD_612 * ADD_612;
  auto ADD_3010 = MUL_2986 + MUL_2989;
  auto MUL_3013 = ADD_3010 * 2.0;
  auto SUB_3016 = 1.0 - MUL_3013;
  auto MUL_3051 = SUB_3016 * 0.0106934;
  auto ADD_3065 = MUL_3039 + MUL_3051;
  auto SUB_3070 = MUL_3060 - ADD_3065;
  auto ADD_3073 = ADD_456 + SUB_3070;
  auto SUB_3004 = MUL_2992 - MUL_2988;
  auto ADD_3017 = MUL_2993 + MUL_2990;
  auto ADD_3026 = MUL_2985 + MUL_2989;
  auto MUL_3029 = ADD_3026 * 2.0;
  auto SUB_3032 = 1.0 - MUL_3029;
  auto MUL_3062 = SUB_3032 * 0.0541677;
  auto MUL_3019 = ADD_3017 * 2.0;
  auto MUL_3055 = MUL_3019 * 0.0106934;
  auto MUL_3006 = SUB_3004 * 2.0;
  auto MUL_3043 = MUL_3006 * 0.0008364;
  auto ADD_3067 = MUL_3043 + MUL_3055;
  auto SUB_3071 = MUL_3062 - ADD_3067;
  auto ADD_3074 = ADD_457 + SUB_3071;
  auto MUL_3088 = MUL_3009 * 0.1581211;
  auto MUL_3095 = MUL_3022 * 0.0051908;
  auto MUL_3077 = SUB_3000 * 0.0002929;
  auto SUB_3105 = MUL_3088 - MUL_3077;
  auto SUB_3108 = SUB_3105 - MUL_3095;
  auto ADD_3111 = ADD_455 + SUB_3108;
  auto MUL_3099 = MUL_3025 * 0.0051908;
  auto MUL_3090 = SUB_3016 * 0.1581211;
  auto MUL_3081 = MUL_3003 * 0.0002929;
  auto SUB_3106 = MUL_3090 - MUL_3081;
  auto SUB_3109 = SUB_3106 - MUL_3099;
  auto ADD_3112 = ADD_456 + SUB_3109;
  auto MUL_3103 = SUB_3032 * 0.0051908;
  auto MUL_3092 = MUL_3019 * 0.1581211;
  auto MUL_3085 = MUL_3006 * 0.0002929;
  auto SUB_3107 = MUL_3092 - MUL_3085;
  auto SUB_3110 = SUB_3107 - MUL_3103;
  auto ADD_3113 = ADD_457 + SUB_3110;
  auto MUL_3127 = MUL_3009 * 0.0524171;
  auto MUL_3134 = MUL_3022 * 0.0213048;
  auto MUL_3116 = SUB_3000 * 0.0014369;
  auto SUB_3144 = MUL_3127 - MUL_3116;
  auto SUB_3147 = SUB_3144 - MUL_3134;
  auto ADD_3150 = ADD_455 + SUB_3147;
  auto MUL_3138 = MUL_3025 * 0.0213048;
  auto MUL_3129 = SUB_3016 * 0.0524171;
  auto MUL_3120 = MUL_3003 * 0.0014369;
  auto SUB_3145 = MUL_3129 - MUL_3120;
  auto SUB_3148 = SUB_3145 - MUL_3138;
  auto ADD_3151 = ADD_456 + SUB_3148;
  auto MUL_3142 = SUB_3032 * 0.0213048;
  auto MUL_3131 = MUL_3019 * 0.0524171;
  auto MUL_3124 = MUL_3006 * 0.0014369;
  auto SUB_3146 = MUL_3131 - MUL_3124;
  auto SUB_3149 = SUB_3146 - MUL_3142;
  auto ADD_3152 = ADD_457 + SUB_3149;
  auto MUL_3179 = MUL_3022 * 0.0299598;
  auto MUL_3155 = SUB_3000 * 0.0006389;
  auto MUL_3167 = MUL_3009 * 1.19e-05;
  auto ADD_3189 = MUL_3155 + MUL_3167;
  auto ADD_3195 = ADD_3189 + MUL_3179;
  auto SUB_3201 = ADD_455 - ADD_3195;
  auto MUL_3183 = MUL_3025 * 0.0299598;
  auto MUL_3171 = SUB_3016 * 1.19e-05;
  auto MUL_3159 = MUL_3003 * 0.0006389;
  auto ADD_3191 = MUL_3159 + MUL_3171;
  auto ADD_3197 = ADD_3191 + MUL_3183;
  auto SUB_3202 = ADD_456 - ADD_3197;
  auto MUL_3187 = SUB_3032 * 0.0299598;
  auto MUL_3175 = MUL_3019 * 1.19e-05;
  auto MUL_3163 = MUL_3006 * 0.0006389;
  auto ADD_3193 = MUL_3163 + MUL_3175;
  auto ADD_3199 = ADD_3193 + MUL_3187;
  auto SUB_3203 = ADD_457 - ADD_3199;
  if(/*arm_left_link_2_l vs. arm_left_link_4_u*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2804, ADD_2805, ADD_2806, 0.187648, ADD_3072, ADD_3073, ADD_3074, 0.153255)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_3111, ADD_3112, ADD_3113, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_3150, ADD_3151, ADD_3152, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, SUB_3201, SUB_3202, SUB_3203, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_3111, ADD_3112, ADD_3113, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_3150, ADD_3151, ADD_3152, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, SUB_3201, SUB_3202, SUB_3203, 0.080043)){ return false; } } // (321, 447)
  if(/*arm_left_link_4_u*/ sphere_environment_in_collision(environment, ADD_3072, ADD_3073, ADD_3074, 0.153255)){ if(sphere_environment_in_collision(environment, ADD_3111, ADD_3112, ADD_3113, 0.089005)){ return false; }
  if(sphere_environment_in_collision(environment, ADD_3150, ADD_3151, ADD_3152, 0.088818)){ return false; }
  if(sphere_environment_in_collision(environment, SUB_3201, SUB_3202, SUB_3203, 0.080043)){ return false; } } // (447, 447)
  if(/*torso_base_link vs. arm_left_link_4_u*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3072, ADD_3073, ADD_3074, 0.153255)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3111, ADD_3112, ADD_3113, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3150, ADD_3151, ADD_3152, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, SUB_3201, SUB_3202, SUB_3203, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_3111, ADD_3112, ADD_3113, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_3150, ADD_3151, ADD_3152, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, SUB_3201, SUB_3202, SUB_3203, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_3111, ADD_3112, ADD_3113, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_3150, ADD_3151, ADD_3152, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, SUB_3201, SUB_3202, SUB_3203, 0.080043)){ return false; } } // (447, 447)
  if(/*torso_link_b1 vs. arm_left_link_4_u*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_2453, NEGATE_2455, 0.6608069, 0.273417, ADD_3072, ADD_3073, ADD_3074, 0.153255)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_3111, ADD_3112, ADD_3113, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_3150, ADD_3151, ADD_3152, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, SUB_3201, SUB_3202, SUB_3203, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_3111, ADD_3112, ADD_3113, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_3150, ADD_3151, ADD_3152, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, SUB_3201, SUB_3202, SUB_3203, 0.080043)){ return false; } } // (447, 447)
  auto MUL_662 = SUB_629 * 0.7071068;
  auto MUL_677 = ADD_624 * 0.7071068;
  auto MUL_701 = ADD_624 * 0.36;
  auto MUL_711 = SUB_629 * MUL_701;
  auto MUL_675 = SUB_618 * 0.7071068;
  auto SUB_688 = MUL_677 - MUL_675;
  auto ADD_678 = MUL_675 + MUL_677;
  auto MUL_664 = ADD_612 * 0.7071068;
  auto SUB_693 = MUL_662 - MUL_664;
  auto ADD_665 = MUL_662 + MUL_664;
  auto MUL_704 = ADD_612 * 0.36;
  auto MUL_713 = SUB_618 * MUL_704;
  auto SUB_714 = MUL_713 - MUL_711;
  auto MUL_717 = SUB_714 * 2.0;
  auto ADD_740 = ADD_455 + MUL_717;
  auto INPUT_6 = q[6];
  auto DIV_744 = INPUT_6 * 0.5;
  auto SIN_745 = DIV_744.sin();
  auto COS_751 = DIV_744.cos();
  auto MUL_768 = SUB_693 * COS_751;
  auto MUL_763 = SUB_693 * SIN_745;
  auto MUL_766 = SUB_688 * COS_751;
  auto ADD_767 = MUL_763 + MUL_766;
  auto MUL_3213 = ADD_767 * ADD_767;
  auto MUL_771 = SUB_688 * SIN_745;
  auto SUB_772 = MUL_768 - MUL_771;
  auto MUL_3214 = SUB_772 * ADD_767;
  auto MUL_753 = ADD_665 * COS_751;
  auto MUL_758 = ADD_665 * SIN_745;
  auto MUL_760 = ADD_678 * COS_751;
  auto SUB_761 = MUL_760 - MUL_758;
  auto MUL_3215 = SUB_772 * SUB_761;
  auto MUL_3212 = SUB_761 * SUB_761;
  auto ADD_3221 = MUL_3212 + MUL_3213;
  auto MUL_3224 = ADD_3221 * 2.0;
  auto SUB_3227 = 1.0 - MUL_3224;
  auto MUL_3262 = SUB_3227 * 0.000508;
  auto MUL_754 = ADD_678 * SIN_745;
  auto ADD_755 = MUL_753 + MUL_754;
  auto MUL_3219 = ADD_755 * ADD_767;
  auto ADD_3247 = MUL_3219 + MUL_3215;
  auto MUL_3249 = ADD_3247 * 2.0;
  auto MUL_3279 = MUL_3249 * 0.0468617;
  auto MUL_3218 = ADD_755 * SUB_761;
  auto SUB_3234 = MUL_3218 - MUL_3214;
  auto MUL_3236 = SUB_3234 * 2.0;
  auto MUL_3273 = MUL_3236 * 0.001079;
  auto SUB_3284 = MUL_3273 - MUL_3262;
  auto ADD_3287 = SUB_3284 + MUL_3279;
  auto ADD_3290 = ADD_740 + ADD_3287;
  auto ADD_3228 = MUL_3218 + MUL_3214;
  auto MUL_3230 = ADD_3228 * 2.0;
  auto MUL_3266 = MUL_3230 * 0.000508;
  auto MUL_3217 = SUB_772 * ADD_755;
  auto MUL_3220 = SUB_761 * ADD_767;
  auto SUB_3250 = MUL_3220 - MUL_3217;
  auto MUL_3252 = SUB_3250 * 2.0;
  auto MUL_3281 = MUL_3252 * 0.0468617;
  auto MUL_3216 = ADD_755 * ADD_755;
  auto ADD_3237 = MUL_3213 + MUL_3216;
  auto MUL_3240 = ADD_3237 * 2.0;
  auto SUB_3243 = 1.0 - MUL_3240;
  auto MUL_3275 = SUB_3243 * 0.001079;
  auto SUB_3285 = MUL_3275 - MUL_3266;
  auto ADD_3288 = SUB_3285 + MUL_3281;
  auto MUL_723 = ADD_624 * MUL_701;
  auto MUL_721 = ADD_612 * MUL_704;
  auto ADD_725 = MUL_721 + MUL_723;
  auto MUL_728 = ADD_725 * 2.0;
  auto SUB_731 = 0.36 - MUL_728;
  auto ADD_741 = ADD_456 + SUB_731;
  auto ADD_3291 = ADD_741 + ADD_3288;
  auto SUB_3231 = MUL_3219 - MUL_3215;
  auto ADD_3244 = MUL_3220 + MUL_3217;
  auto ADD_3253 = MUL_3212 + MUL_3216;
  auto MUL_3256 = ADD_3253 * 2.0;
  auto SUB_3259 = 1.0 - MUL_3256;
  auto MUL_3283 = SUB_3259 * 0.0468617;
  auto MUL_3246 = ADD_3244 * 2.0;
  auto MUL_3277 = MUL_3246 * 0.001079;
  auto MUL_3233 = SUB_3231 * 2.0;
  auto MUL_3270 = MUL_3233 * 0.000508;
  auto SUB_3286 = MUL_3277 - MUL_3270;
  auto ADD_3289 = SUB_3286 + MUL_3283;
  auto MUL_732 = SUB_629 * MUL_704;
  auto MUL_734 = SUB_618 * MUL_701;
  auto ADD_736 = MUL_732 + MUL_734;
  auto MUL_738 = ADD_736 * 2.0;
  auto ADD_742 = ADD_457 + MUL_738;
  auto ADD_3292 = ADD_742 + ADD_3289;
  auto MUL_3312 = MUL_3249 * 0.1007051;
  auto MUL_3301 = MUL_3236 * 0.0017953;
  auto MUL_3294 = SUB_3227 * 0.0003948;
  auto SUB_3317 = MUL_3294 - MUL_3301;
  auto ADD_3320 = SUB_3317 + MUL_3312;
  auto ADD_3323 = ADD_740 + ADD_3320;
  auto MUL_3314 = MUL_3252 * 0.1007051;
  auto MUL_3305 = SUB_3243 * 0.0017953;
  auto MUL_3296 = MUL_3230 * 0.0003948;
  auto SUB_3318 = MUL_3296 - MUL_3305;
  auto ADD_3321 = SUB_3318 + MUL_3314;
  auto ADD_3324 = ADD_741 + ADD_3321;
  auto MUL_3316 = SUB_3259 * 0.1007051;
  auto MUL_3309 = MUL_3246 * 0.0017953;
  auto MUL_3298 = MUL_3233 * 0.0003948;
  auto SUB_3319 = MUL_3298 - MUL_3309;
  auto ADD_3322 = SUB_3319 + MUL_3316;
  auto ADD_3325 = ADD_742 + ADD_3322;
  auto MUL_3340 = MUL_3236 * 0.0169963;
  auto MUL_3351 = MUL_3249 * 0.0120211;
  auto MUL_3328 = SUB_3227 * 0.0010222;
  auto ADD_3356 = MUL_3328 + MUL_3340;
  auto SUB_3362 = MUL_3351 - ADD_3356;
  auto ADD_3365 = ADD_740 + SUB_3362;
  auto MUL_3353 = MUL_3252 * 0.0120211;
  auto MUL_3344 = SUB_3243 * 0.0169963;
  auto MUL_3332 = MUL_3230 * 0.0010222;
  auto ADD_3358 = MUL_3332 + MUL_3344;
  auto SUB_3363 = MUL_3353 - ADD_3358;
  auto ADD_3366 = ADD_741 + SUB_3363;
  auto MUL_3355 = SUB_3259 * 0.0120211;
  auto MUL_3348 = MUL_3246 * 0.0169963;
  auto MUL_3336 = MUL_3233 * 0.0010222;
  auto ADD_3360 = MUL_3336 + MUL_3348;
  auto SUB_3364 = MUL_3355 - ADD_3360;
  auto ADD_3367 = ADD_742 + SUB_3364;
  if(/*arm_left_link_2_l vs. arm_left_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2804, ADD_2805, ADD_2806, 0.187648, ADD_3290, ADD_3291, ADD_3292, 0.095701)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_3323, ADD_3324, ADD_3325, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_3365, ADD_3366, ADD_3367, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_3323, ADD_3324, ADD_3325, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_3365, ADD_3366, ADD_3367, 0.077046)){ return false; } } // (447, 573)
  if(/*arm_left_link_5_r*/ sphere_environment_in_collision(environment, ADD_3290, ADD_3291, ADD_3292, 0.095701)){ if(sphere_environment_in_collision(environment, ADD_3323, ADD_3324, ADD_3325, 0.079093)){ return false; }
  if(sphere_environment_in_collision(environment, ADD_3365, ADD_3366, ADD_3367, 0.077046)){ return false; } } // (573, 573)
  if(/*torso_base_link vs. arm_left_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3290, ADD_3291, ADD_3292, 0.095701)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3323, ADD_3324, ADD_3325, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3365, ADD_3366, ADD_3367, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_3323, ADD_3324, ADD_3325, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_3365, ADD_3366, ADD_3367, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_3323, ADD_3324, ADD_3325, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_3365, ADD_3366, ADD_3367, 0.077046)){ return false; } } // (573, 573)
  if(/*torso_link_b1 vs. arm_left_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_2453, NEGATE_2455, 0.6608069, 0.273417, ADD_3290, ADD_3291, ADD_3292, 0.095701)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_3323, ADD_3324, ADD_3325, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_3365, ADD_3366, ADD_3367, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_3323, ADD_3324, ADD_3325, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_3365, ADD_3366, ADD_3367, 0.077046)){ return false; } } // (573, 573)
  auto MUL_806 = SUB_772 * 0.7071068;
  auto MUL_823 = ADD_767 * 0.7071068;
  auto MUL_820 = SUB_761 * 0.7071068;
  auto SUB_825 = MUL_820 - MUL_823;
  auto ADD_836 = MUL_820 + MUL_823;
  auto MUL_809 = ADD_755 * 0.7071068;
  auto SUB_810 = MUL_809 - MUL_806;
  auto ADD_843 = MUL_806 + MUL_809;
  auto INPUT_7 = q[7];
  auto DIV_880 = INPUT_7 * 0.5;
  auto SIN_881 = DIV_880.sin();
  auto COS_887 = DIV_880.cos();
  auto MUL_899 = ADD_843 * SIN_881;
  auto MUL_904 = ADD_843 * COS_887;
  auto MUL_907 = ADD_836 * SIN_881;
  auto SUB_908 = MUL_904 - MUL_907;
  auto MUL_902 = ADD_836 * COS_887;
  auto ADD_903 = MUL_899 + MUL_902;
  auto MUL_3376 = SUB_908 * ADD_903;
  auto MUL_3375 = ADD_903 * ADD_903;
  auto MUL_894 = SUB_810 * SIN_881;
  auto MUL_889 = SUB_810 * COS_887;
  auto MUL_890 = SUB_825 * SIN_881;
  auto ADD_891 = MUL_889 + MUL_890;
  auto MUL_3381 = ADD_891 * ADD_903;
  auto MUL_896 = SUB_825 * COS_887;
  auto SUB_897 = MUL_896 - MUL_894;
  auto MUL_3377 = SUB_908 * SUB_897;
  auto ADD_3409 = MUL_3381 + MUL_3377;
  auto MUL_3411 = ADD_3409 * 2.0;
  auto MUL_3435 = MUL_3411 * 0.0173577;
  auto MUL_3374 = SUB_897 * SUB_897;
  auto ADD_3383 = MUL_3374 + MUL_3375;
  auto MUL_3386 = ADD_3383 * 2.0;
  auto SUB_3389 = 1.0 - MUL_3386;
  auto MUL_3423 = SUB_3389 * 0.002951;
  auto MUL_3380 = ADD_891 * SUB_897;
  auto SUB_3396 = MUL_3380 - MUL_3376;
  auto MUL_3398 = SUB_3396 * 2.0;
  auto MUL_3429 = MUL_3398 * 0.106206;
  auto ADD_3440 = MUL_3423 + MUL_3429;
  auto ADD_3443 = ADD_3440 + MUL_3435;
  auto ADD_3446 = ADD_740 + ADD_3443;
  auto ADD_3390 = MUL_3380 + MUL_3376;
  auto MUL_3392 = ADD_3390 * 2.0;
  auto MUL_3425 = MUL_3392 * 0.002951;
  auto MUL_3379 = SUB_908 * ADD_891;
  auto MUL_3382 = SUB_897 * ADD_903;
  auto SUB_3412 = MUL_3382 - MUL_3379;
  auto MUL_3414 = SUB_3412 * 2.0;
  auto MUL_3437 = MUL_3414 * 0.0173577;
  auto MUL_3378 = ADD_891 * ADD_891;
  auto ADD_3399 = MUL_3375 + MUL_3378;
  auto MUL_3402 = ADD_3399 * 2.0;
  auto SUB_3405 = 1.0 - MUL_3402;
  auto MUL_3431 = SUB_3405 * 0.106206;
  auto ADD_3441 = MUL_3425 + MUL_3431;
  auto ADD_3444 = ADD_3441 + MUL_3437;
  auto ADD_3447 = ADD_741 + ADD_3444;
  auto SUB_3393 = MUL_3381 - MUL_3377;
  auto ADD_3406 = MUL_3382 + MUL_3379;
  auto ADD_3415 = MUL_3374 + MUL_3378;
  auto MUL_3418 = ADD_3415 * 2.0;
  auto SUB_3421 = 1.0 - MUL_3418;
  auto MUL_3439 = SUB_3421 * 0.0173577;
  auto MUL_3408 = ADD_3406 * 2.0;
  auto MUL_3433 = MUL_3408 * 0.106206;
  auto MUL_3395 = SUB_3393 * 2.0;
  auto MUL_3427 = MUL_3395 * 0.002951;
  auto ADD_3442 = MUL_3427 + MUL_3433;
  auto ADD_3445 = ADD_3442 + MUL_3439;
  auto ADD_3448 = ADD_742 + ADD_3445;
  auto MUL_3462 = MUL_3398 * 0.0819007;
  auto MUL_3468 = MUL_3411 * 0.0063689;
  auto MUL_3451 = SUB_3389 * 1.14e-05;
  auto SUB_3473 = MUL_3462 - MUL_3451;
  auto ADD_3476 = SUB_3473 + MUL_3468;
  auto ADD_3479 = ADD_740 + ADD_3476;
  auto MUL_3470 = MUL_3414 * 0.0063689;
  auto MUL_3464 = SUB_3405 * 0.0819007;
  auto MUL_3455 = MUL_3392 * 1.14e-05;
  auto SUB_3474 = MUL_3464 - MUL_3455;
  auto ADD_3477 = SUB_3474 + MUL_3470;
  auto ADD_3480 = ADD_741 + ADD_3477;
  auto MUL_3472 = SUB_3421 * 0.0063689;
  auto MUL_3466 = MUL_3408 * 0.0819007;
  auto MUL_3459 = MUL_3395 * 1.14e-05;
  auto SUB_3475 = MUL_3466 - MUL_3459;
  auto ADD_3478 = SUB_3475 + MUL_3472;
  auto ADD_3481 = ADD_742 + ADD_3478;
  auto MUL_3501 = MUL_3411 * 0.0259799;
  auto MUL_3495 = MUL_3398 * 0.0086577;
  auto MUL_3484 = SUB_3389 * 0.0001404;
  auto SUB_3506 = MUL_3495 - MUL_3484;
  auto ADD_3509 = SUB_3506 + MUL_3501;
  auto ADD_3512 = ADD_740 + ADD_3509;
  auto MUL_3503 = MUL_3414 * 0.0259799;
  auto MUL_3497 = SUB_3405 * 0.0086577;
  auto MUL_3488 = MUL_3392 * 0.0001404;
  auto SUB_3507 = MUL_3497 - MUL_3488;
  auto ADD_3510 = SUB_3507 + MUL_3503;
  auto ADD_3513 = ADD_741 + ADD_3510;
  auto MUL_3505 = SUB_3421 * 0.0259799;
  auto MUL_3499 = MUL_3408 * 0.0086577;
  auto MUL_3492 = MUL_3395 * 0.0001404;
  auto SUB_3508 = MUL_3499 - MUL_3492;
  auto ADD_3511 = SUB_3508 + MUL_3505;
  auto ADD_3514 = ADD_742 + ADD_3511;
  if(/*arm_left_link_2_l vs. arm_left_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2804, ADD_2805, ADD_2806, 0.187648, ADD_3446, ADD_3447, ADD_3448, 0.119311)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_3479, ADD_3480, ADD_3481, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_3512, ADD_3513, ADD_3514, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_3479, ADD_3480, ADD_3481, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_3512, ADD_3513, ADD_3514, 0.068019)){ return false; } } // (573, 681)
  if(/*arm_left_link_6_b*/ sphere_environment_in_collision(environment, ADD_3446, ADD_3447, ADD_3448, 0.119311)){ if(sphere_environment_in_collision(environment, ADD_3479, ADD_3480, ADD_3481, 0.077881)){ return false; }
  if(sphere_environment_in_collision(environment, ADD_3512, ADD_3513, ADD_3514, 0.068019)){ return false; } } // (681, 681)
  if(/*torso_base_link vs. arm_left_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3446, ADD_3447, ADD_3448, 0.119311)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3479, ADD_3480, ADD_3481, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3512, ADD_3513, ADD_3514, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_3479, ADD_3480, ADD_3481, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_3512, ADD_3513, ADD_3514, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_3479, ADD_3480, ADD_3481, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_3512, ADD_3513, ADD_3514, 0.068019)){ return false; } } // (681, 681)
  if(/*torso_link_b1 vs. arm_left_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_2453, NEGATE_2455, 0.6608069, 0.273417, ADD_3446, ADD_3447, ADD_3448, 0.119311)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_3479, ADD_3480, ADD_3481, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_3512, ADD_3513, ADD_3514, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_3479, ADD_3480, ADD_3481, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_3512, ADD_3513, ADD_3514, 0.068019)){ return false; } } // (681, 681)
  auto MUL_942 = SUB_908 * 0.7071068;
  auto MUL_959 = ADD_903 * 0.7071068;
  auto MUL_956 = SUB_897 * 0.7071068;
  auto SUB_961 = MUL_956 - MUL_959;
  auto ADD_972 = MUL_956 + MUL_959;
  auto MUL_945 = ADD_891 * 0.7071068;
  auto SUB_946 = MUL_945 - MUL_942;
  auto ADD_979 = MUL_942 + MUL_945;
  auto MUL_987 = ADD_903 * 0.155;
  auto MUL_997 = SUB_908 * MUL_987;
  auto MUL_990 = ADD_891 * 0.155;
  auto MUL_999 = SUB_897 * MUL_990;
  auto SUB_1000 = MUL_999 - MUL_997;
  auto MUL_1003 = SUB_1000 * 2.0;
  auto ADD_1026 = ADD_740 + MUL_1003;
  auto INPUT_8 = q[8];
  auto DIV_1030 = INPUT_8 * 0.5;
  auto SIN_1031 = DIV_1030.sin();
  auto COS_1040 = DIV_1030.cos();
  auto MUL_1038 = SIN_1031 * 1.0;
  auto MUL_1053 = ADD_979 * MUL_1038;
  auto MUL_1059 = ADD_979 * COS_1040;
  auto MUL_1062 = ADD_972 * MUL_1038;
  auto ADD_1064 = MUL_1059 + MUL_1062;
  auto MUL_1057 = ADD_972 * COS_1040;
  auto SUB_1058 = MUL_1057 - MUL_1053;
  auto MUL_3523 = ADD_1064 * SUB_1058;
  auto MUL_3522 = SUB_1058 * SUB_1058;
  auto MUL_1048 = SUB_946 * MUL_1038;
  auto MUL_1042 = SUB_946 * COS_1040;
  auto MUL_1043 = SUB_961 * MUL_1038;
  auto SUB_1045 = MUL_1042 - MUL_1043;
  auto MUL_3528 = SUB_1045 * SUB_1058;
  auto MUL_1050 = SUB_961 * COS_1040;
  auto ADD_1051 = MUL_1048 + MUL_1050;
  auto MUL_3524 = ADD_1064 * ADD_1051;
  auto ADD_3556 = MUL_3528 + MUL_3524;
  auto MUL_3558 = ADD_3556 * 2.0;
  auto MUL_3582 = MUL_3558 * 0.116149;
  auto MUL_3527 = SUB_1045 * ADD_1051;
  auto SUB_3543 = MUL_3527 - MUL_3523;
  auto MUL_3545 = SUB_3543 * 2.0;
  auto MUL_3576 = MUL_3545 * 0.0035861;
  auto MUL_3521 = ADD_1051 * ADD_1051;
  auto ADD_3530 = MUL_3521 + MUL_3522;
  auto MUL_3533 = ADD_3530 * 2.0;
  auto SUB_3536 = 1.0 - MUL_3533;
  auto MUL_3570 = SUB_3536 * 0.0006352;
  auto ADD_3587 = MUL_3570 + MUL_3576;
  auto ADD_3590 = ADD_3587 + MUL_3582;
  auto ADD_3593 = ADD_1026 + ADD_3590;
  auto ADD_3537 = MUL_3527 + MUL_3523;
  auto MUL_3539 = ADD_3537 * 2.0;
  auto MUL_3572 = MUL_3539 * 0.0006352;
  auto MUL_3526 = ADD_1064 * SUB_1045;
  auto MUL_3525 = SUB_1045 * SUB_1045;
  auto ADD_3546 = MUL_3522 + MUL_3525;
  auto MUL_3549 = ADD_3546 * 2.0;
  auto SUB_3552 = 1.0 - MUL_3549;
  auto MUL_3578 = SUB_3552 * 0.0035861;
  auto ADD_3588 = MUL_3572 + MUL_3578;
  auto MUL_3529 = ADD_1051 * SUB_1058;
  auto SUB_3559 = MUL_3529 - MUL_3526;
  auto MUL_3561 = SUB_3559 * 2.0;
  auto MUL_3584 = MUL_3561 * 0.116149;
  auto ADD_3591 = ADD_3588 + MUL_3584;
  auto MUL_1009 = ADD_903 * MUL_987;
  auto MUL_1007 = ADD_891 * MUL_990;
  auto ADD_1011 = MUL_1007 + MUL_1009;
  auto MUL_1014 = ADD_1011 * 2.0;
  auto SUB_1017 = 0.155 - MUL_1014;
  auto ADD_1027 = ADD_741 + SUB_1017;
  auto ADD_3594 = ADD_1027 + ADD_3591;
  auto SUB_3540 = MUL_3528 - MUL_3524;
  auto ADD_3553 = MUL_3529 + MUL_3526;
  auto ADD_3562 = MUL_3521 + MUL_3525;
  auto MUL_3565 = ADD_3562 * 2.0;
  auto SUB_3568 = 1.0 - MUL_3565;
  auto MUL_3586 = SUB_3568 * 0.116149;
  auto MUL_3555 = ADD_3553 * 2.0;
  auto MUL_3580 = MUL_3555 * 0.0035861;
  auto MUL_3542 = SUB_3540 * 2.0;
  auto MUL_3574 = MUL_3542 * 0.0006352;
  auto ADD_3589 = MUL_3574 + MUL_3580;
  auto ADD_3592 = ADD_3589 + MUL_3586;
  auto MUL_1018 = SUB_908 * MUL_990;
  auto MUL_1020 = SUB_897 * MUL_987;
  auto ADD_1022 = MUL_1018 + MUL_1020;
  auto MUL_1024 = ADD_1022 * 2.0;
  auto ADD_1028 = ADD_742 + MUL_1024;
  auto ADD_3595 = ADD_1028 + ADD_3592;
  auto MUL_3609 = MUL_3545 * 0.018019;
  auto MUL_3616 = MUL_3558 * 0.00659;
  auto MUL_3598 = SUB_3536 * 0.004017;
  auto SUB_3626 = MUL_3609 - MUL_3598;
  auto SUB_3629 = SUB_3626 - MUL_3616;
  auto ADD_3632 = ADD_1026 + SUB_3629;
  auto MUL_3620 = MUL_3561 * 0.00659;
  auto MUL_3611 = SUB_3552 * 0.018019;
  auto MUL_3602 = MUL_3539 * 0.004017;
  auto SUB_3627 = MUL_3611 - MUL_3602;
  auto SUB_3630 = SUB_3627 - MUL_3620;
  auto ADD_3633 = ADD_1027 + SUB_3630;
  auto MUL_3624 = SUB_3568 * 0.00659;
  auto MUL_3613 = MUL_3555 * 0.018019;
  auto MUL_3606 = MUL_3542 * 0.004017;
  auto SUB_3628 = MUL_3613 - MUL_3606;
  auto SUB_3631 = SUB_3628 - MUL_3624;
  auto ADD_3634 = ADD_1028 + SUB_3631;
  auto MUL_3636 = SUB_3536 * 0.017613;
  auto MUL_3655 = MUL_3558 * 0.006585;
  auto MUL_3643 = MUL_3545 * 0.005894;
  auto SUB_3665 = MUL_3636 - MUL_3643;
  auto SUB_3668 = SUB_3665 - MUL_3655;
  auto ADD_3671 = ADD_1026 + SUB_3668;
  auto MUL_3659 = MUL_3561 * 0.006585;
  auto MUL_3647 = SUB_3552 * 0.005894;
  auto MUL_3638 = MUL_3539 * 0.017613;
  auto SUB_3666 = MUL_3638 - MUL_3647;
  auto SUB_3669 = SUB_3666 - MUL_3659;
  auto ADD_3672 = ADD_1027 + SUB_3669;
  auto MUL_3663 = SUB_3568 * 0.006585;
  auto MUL_3651 = MUL_3555 * 0.005894;
  auto MUL_3640 = MUL_3542 * 0.017613;
  auto SUB_3667 = MUL_3640 - MUL_3651;
  auto SUB_3670 = SUB_3667 - MUL_3663;
  auto ADD_3673 = ADD_1028 + SUB_3670;
  auto MUL_3676 = SUB_3536 * 0.01441;
  auto MUL_3688 = MUL_3545 * 0.012936;
  auto ADD_3710 = MUL_3676 + MUL_3688;
  auto MUL_3700 = MUL_3558 * 0.006545;
  auto ADD_3716 = ADD_3710 + MUL_3700;
  auto SUB_3722 = ADD_1026 - ADD_3716;
  auto MUL_3704 = MUL_3561 * 0.006545;
  auto MUL_3692 = SUB_3552 * 0.012936;
  auto MUL_3680 = MUL_3539 * 0.01441;
  auto ADD_3712 = MUL_3680 + MUL_3692;
  auto ADD_3718 = ADD_3712 + MUL_3704;
  auto SUB_3723 = ADD_1027 - ADD_3718;
  auto MUL_3708 = SUB_3568 * 0.006545;
  auto MUL_3696 = MUL_3555 * 0.012936;
  auto MUL_3684 = MUL_3542 * 0.01441;
  auto ADD_3714 = MUL_3684 + MUL_3696;
  auto ADD_3720 = ADD_3714 + MUL_3708;
  auto SUB_3724 = ADD_1028 - ADD_3720;
  if(/*arm_left_link_2_l vs. arm_left_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2804, ADD_2805, ADD_2806, 0.187648, ADD_3593, ADD_3594, ADD_3595, 0.095372)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_3632, ADD_3633, ADD_3634, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_3671, ADD_3672, ADD_3673, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, SUB_3722, SUB_3723, SUB_3724, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_3632, ADD_3633, ADD_3634, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_3671, ADD_3672, ADD_3673, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, SUB_3722, SUB_3723, SUB_3724, 0.034568)){ return false; } } // (681, 826)
  if(/*arm_left_link_7_t*/ sphere_environment_in_collision(environment, ADD_3593, ADD_3594, ADD_3595, 0.095372)){ if(sphere_environment_in_collision(environment, ADD_3632, ADD_3633, ADD_3634, 0.035041)){ return false; }
  if(sphere_environment_in_collision(environment, ADD_3671, ADD_3672, ADD_3673, 0.035213)){ return false; }
  if(sphere_environment_in_collision(environment, SUB_3722, SUB_3723, SUB_3724, 0.034568)){ return false; } } // (826, 826)
  if(/*torso_base_link vs. arm_left_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3593, ADD_3594, ADD_3595, 0.095372)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3632, ADD_3633, ADD_3634, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_3671, ADD_3672, ADD_3673, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, SUB_3722, SUB_3723, SUB_3724, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_3632, ADD_3633, ADD_3634, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_3671, ADD_3672, ADD_3673, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, SUB_3722, SUB_3723, SUB_3724, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_3632, ADD_3633, ADD_3634, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_3671, ADD_3672, ADD_3673, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, SUB_3722, SUB_3723, SUB_3724, 0.034568)){ return false; } } // (826, 826)
  if(/*torso_link_b1 vs. arm_left_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_2453, NEGATE_2455, 0.6608069, 0.273417, ADD_3593, ADD_3594, ADD_3595, 0.095372)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_3632, ADD_3633, ADD_3634, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_3671, ADD_3672, ADD_3673, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, SUB_3722, SUB_3723, SUB_3724, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_3632, ADD_3633, ADD_3634, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_3671, ADD_3672, ADD_3673, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, SUB_3722, SUB_3723, SUB_3724, 0.034568)){ return false; } } // (826, 826)
  auto SUB_1279 = MUL_136 - MUL_139;
  auto MUL_1281 = SUB_1279 * 2.0;
  auto ADD_1283 = MUL_1281 + 0.1;
  auto INPUT_9 = q[9];
  auto DIV_1304 = INPUT_9 * 0.5;
  auto SIN_1305 = DIV_1304.sin();
  auto COS_1311 = DIV_1304.cos();
  auto MUL_1321 = MUL_115 * COS_1311;
  auto MUL_1335 = MUL_115 * SIN_1305;
  auto MUL_1332 = MUL_113 * COS_1311;
  auto SUB_1316 = MUL_1332 - MUL_1335;
  auto ADD_1337 = MUL_1332 + MUL_1335;
  auto MUL_1319 = MUL_113 * SIN_1305;
  auto SUB_1331 = MUL_1319 - MUL_1321;
  auto ADD_1323 = MUL_1319 + MUL_1321;
  auto MUL_3825 = ADD_1337 * SUB_1331;
  auto MUL_3826 = ADD_1337 * ADD_1323;
  auto MUL_3824 = SUB_1331 * SUB_1331;
  auto MUL_3823 = ADD_1323 * ADD_1323;
  auto ADD_3835 = MUL_3823 + MUL_3824;
  auto MUL_3838 = ADD_3835 * 2.0;
  auto SUB_3841 = 1.0 - MUL_3838;
  auto MUL_3921 = SUB_3841 * 0.0008364;
  auto MUL_3832 = SUB_1316 * SUB_1331;
  auto SUB_3863 = MUL_3832 - MUL_3826;
  auto MUL_3865 = SUB_3863 * 2.0;
  auto MUL_3943 = MUL_3865 * 0.0541677;
  auto MUL_3830 = SUB_1316 * ADD_1323;
  auto ADD_3848 = MUL_3830 + MUL_3825;
  auto MUL_3851 = ADD_3848 * 2.0;
  auto MUL_3933 = MUL_3851 * 0.0106934;
  auto SUB_3949 = MUL_3933 - MUL_3921;
  auto ADD_3954 = SUB_3949 + MUL_3943;
  auto ADD_3958 = ADD_1283 + ADD_3954;
  auto SUB_3842 = MUL_3825 - MUL_3830;
  auto ADD_1288 = MUL_149 + MUL_151;
  auto MUL_3844 = SUB_3842 * 2.0;
  auto MUL_3925 = MUL_3844 * 0.0008364;
  auto MUL_3829 = ADD_1337 * SUB_1316;
  auto MUL_3833 = ADD_1323 * SUB_1331;
  auto ADD_3866 = MUL_3833 + MUL_3829;
  auto MUL_3869 = ADD_3866 * 2.0;
  auto MUL_3945 = MUL_3869 * 0.0541677;
  auto MUL_3828 = SUB_1316 * SUB_1316;
  auto ADD_3853 = MUL_3824 + MUL_3828;
  auto MUL_3856 = ADD_3853 * 2.0;
  auto SUB_3859 = 1.0 - MUL_3856;
  auto MUL_3936 = SUB_3859 * 0.0106934;
  auto ADD_3950 = MUL_3925 + MUL_3936;
  auto ADD_3955 = ADD_3950 + MUL_3945;
  auto MUL_1290 = ADD_1288 * 2.0;
  auto SUB_1293 = MUL_1290 - 0.265;
  auto SUB_3959 = SUB_1293 - ADD_3955;
  auto SUB_3860 = MUL_3829 - MUL_3833;
  auto ADD_3845 = MUL_3832 + MUL_3826;
  auto ADD_3871 = MUL_3823 + MUL_3828;
  auto MUL_3874 = ADD_3871 * 2.0;
  auto SUB_3877 = 1.0 - MUL_3874;
  auto MUL_3948 = SUB_3877 * 0.0541677;
  auto MUL_3862 = SUB_3860 * 2.0;
  auto MUL_3940 = MUL_3862 * 0.0106934;
  auto MUL_3847 = ADD_3845 * 2.0;
  auto MUL_3929 = MUL_3847 * 0.0008364;
  auto ADD_3952 = MUL_3929 + MUL_3940;
  auto SUB_3957 = MUL_3948 - ADD_3952;
  auto ADD_3960 = 1.2 + SUB_3957;
  if(/*arm_right_link_1_s*/ sphere_environment_in_collision(environment, ADD_3958, SUB_3959, ADD_3960, 0.153255)){ return false; } // (826, 892)
  auto MUL_1371 = ADD_1337 * 0.7071068;
  auto MUL_1389 = SUB_1331 * 0.7071068;
  auto MUL_1385 = ADD_1323 * 0.7071068;
  auto SUB_1403 = MUL_1389 - MUL_1385;
  auto ADD_1391 = MUL_1385 + MUL_1389;
  auto MUL_1374 = SUB_1316 * 0.7071068;
  auto SUB_1375 = MUL_1374 - MUL_1371;
  auto ADD_1410 = MUL_1371 + MUL_1374;
  auto INPUT_10 = q[10];
  auto DIV_1447 = INPUT_10 * 0.5;
  auto SIN_1448 = DIV_1447.sin();
  auto COS_1454 = DIV_1447.cos();
  auto MUL_1469 = ADD_1410 * SIN_1448;
  auto MUL_1474 = ADD_1410 * COS_1454;
  auto MUL_1477 = SUB_1403 * SIN_1448;
  auto SUB_1478 = MUL_1474 - MUL_1477;
  auto MUL_1472 = SUB_1403 * COS_1454;
  auto ADD_1473 = MUL_1469 + MUL_1472;
  auto MUL_3965 = SUB_1478 * ADD_1473;
  auto MUL_3964 = ADD_1473 * ADD_1473;
  auto MUL_1462 = SUB_1375 * SIN_1448;
  auto MUL_1456 = SUB_1375 * COS_1454;
  auto MUL_1457 = ADD_1391 * SIN_1448;
  auto SUB_1459 = MUL_1456 - MUL_1457;
  auto MUL_3972 = SUB_1459 * ADD_1473;
  auto MUL_1464 = ADD_1391 * COS_1454;
  auto ADD_1466 = MUL_1462 + MUL_1464;
  auto MUL_3966 = SUB_1478 * ADD_1466;
  auto SUB_4003 = MUL_3972 - MUL_3966;
  auto MUL_4005 = SUB_4003 * 2.0;
  auto MUL_4039 = MUL_4005 * 0.0213048;
  auto MUL_3963 = ADD_1466 * ADD_1466;
  auto ADD_3975 = MUL_3963 + MUL_3964;
  auto MUL_3978 = ADD_3975 * 2.0;
  auto SUB_3981 = 1.0 - MUL_3978;
  auto MUL_4020 = SUB_3981 * 0.0014369;
  auto MUL_3970 = SUB_1459 * ADD_1466;
  auto ADD_3988 = MUL_3970 + MUL_3965;
  auto MUL_3991 = ADD_3988 * 2.0;
  auto MUL_4031 = MUL_3991 * 0.0524171;
  auto ADD_4048 = MUL_4020 + MUL_4031;
  auto ADD_4052 = ADD_4048 + MUL_4039;
  auto SUB_4056 = ADD_1283 - ADD_4052;
  auto SUB_3982 = MUL_3965 - MUL_3970;
  auto MUL_3984 = SUB_3982 * 2.0;
  auto MUL_4024 = MUL_3984 * 0.0014369;
  auto MUL_3969 = SUB_1478 * SUB_1459;
  auto MUL_3973 = ADD_1466 * ADD_1473;
  auto ADD_4006 = MUL_3973 + MUL_3969;
  auto MUL_4009 = ADD_4006 * 2.0;
  auto MUL_4043 = MUL_4009 * 0.0213048;
  auto MUL_3968 = SUB_1459 * SUB_1459;
  auto ADD_3993 = MUL_3964 + MUL_3968;
  auto MUL_3996 = ADD_3993 * 2.0;
  auto SUB_3999 = 1.0 - MUL_3996;
  auto MUL_4034 = SUB_3999 * 0.0524171;
  auto SUB_4050 = MUL_4034 - MUL_4024;
  auto ADD_4054 = SUB_4050 + MUL_4043;
  auto ADD_4057 = SUB_1293 + ADD_4054;
  auto SUB_4000 = MUL_3969 - MUL_3973;
  auto ADD_3985 = MUL_3972 + MUL_3966;
  auto ADD_4011 = MUL_3963 + MUL_3968;
  auto MUL_4014 = ADD_4011 * 2.0;
  auto SUB_4017 = 1.0 - MUL_4014;
  auto MUL_4046 = SUB_4017 * 0.0213048;
  auto MUL_4002 = SUB_4000 * 2.0;
  auto MUL_4036 = MUL_4002 * 0.0524171;
  auto MUL_3987 = ADD_3985 * 2.0;
  auto MUL_4028 = MUL_3987 * 0.0014369;
  auto SUB_4051 = MUL_4036 - MUL_4028;
  auto SUB_4055 = SUB_4051 - MUL_4046;
  auto ADD_4058 = 1.2 + SUB_4055;
  auto MUL_4079 = MUL_4005 * 0.0468617;
  auto MUL_4072 = MUL_3991 * 0.001079;
  auto MUL_4061 = SUB_3981 * 0.000508;
  auto ADD_4085 = MUL_4061 + MUL_4072;
  auto SUB_4089 = MUL_4079 - ADD_4085;
  auto ADD_4092 = ADD_1283 + SUB_4089;
  auto MUL_4081 = MUL_4009 * 0.0468617;
  auto MUL_4075 = SUB_3999 * 0.001079;
  auto MUL_4065 = MUL_3984 * 0.000508;
  auto SUB_4087 = MUL_4075 - MUL_4065;
  auto SUB_4090 = SUB_4087 - MUL_4081;
  auto ADD_4093 = SUB_1293 + SUB_4090;
  auto MUL_4084 = SUB_4017 * 0.0468617;
  auto MUL_4077 = MUL_4002 * 0.001079;
  auto MUL_4069 = MUL_3987 * 0.000508;
  auto SUB_4088 = MUL_4077 - MUL_4069;
  auto ADD_4091 = SUB_4088 + MUL_4084;
  auto ADD_4094 = 1.2 + ADD_4091;
  auto MUL_4109 = MUL_4005 * 0.0173577;
  auto MUL_4102 = MUL_3991 * 0.106206;
  auto MUL_4096 = SUB_3981 * 0.002951;
  auto SUB_4115 = MUL_4096 - MUL_4102;
  auto ADD_4118 = SUB_4115 + MUL_4109;
  auto ADD_4121 = ADD_1283 + ADD_4118;
  auto MUL_4111 = MUL_4009 * 0.0173577;
  auto MUL_4105 = SUB_3999 * 0.106206;
  auto MUL_4098 = MUL_3984 * 0.002951;
  auto ADD_4116 = MUL_4098 + MUL_4105;
  auto SUB_4119 = ADD_4116 - MUL_4111;
  auto ADD_4122 = SUB_1293 + SUB_4119;
  auto MUL_4114 = SUB_4017 * 0.0173577;
  auto MUL_4107 = MUL_4002 * 0.106206;
  auto MUL_4100 = MUL_3987 * 0.002951;
  auto ADD_4117 = MUL_4100 + MUL_4107;
  auto ADD_4120 = ADD_4117 + MUL_4114;
  auto ADD_4123 = 1.2 + ADD_4120;
  if(/*arm_left_link_5_r vs. arm_right_link_2_l*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3290, ADD_3291, ADD_3292, 0.095701, SUB_4056, ADD_4057, ADD_4058, 0.088818)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4092, ADD_4093, ADD_4094, 0.095701)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4121, ADD_4122, ADD_4123, 0.119311)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4092, ADD_4093, ADD_4094, 0.095701)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4121, ADD_4122, ADD_4123, 0.119311)){ return false; } } // (892, 1000)
  if(/*arm_left_link_6_b vs. arm_right_link_2_l*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3446, ADD_3447, ADD_3448, 0.119311, SUB_4056, ADD_4057, ADD_4058, 0.088818)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4092, ADD_4093, ADD_4094, 0.095701)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4121, ADD_4122, ADD_4123, 0.119311)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4092, ADD_4093, ADD_4094, 0.095701)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4121, ADD_4122, ADD_4123, 0.119311)){ return false; } } // (1000, 1000)
  if(/*arm_left_link_7_t vs. arm_right_link_2_l*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3593, ADD_3594, ADD_3595, 0.095372, SUB_4056, ADD_4057, ADD_4058, 0.088818)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4092, ADD_4093, ADD_4094, 0.095701)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4121, ADD_4122, ADD_4123, 0.119311)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4092, ADD_4093, ADD_4094, 0.095701)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4121, ADD_4122, ADD_4123, 0.119311)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4092, ADD_4093, ADD_4094, 0.095701)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4121, ADD_4122, ADD_4123, 0.119311)){ return false; } } // (1000, 1000)
  if(/*arm_right_link_2_l*/ sphere_environment_in_collision(environment, SUB_4056, ADD_4057, ADD_4058, 0.088818)){ if(sphere_environment_in_collision(environment, ADD_4092, ADD_4093, ADD_4094, 0.095701)){ return false; }
  if(sphere_environment_in_collision(environment, ADD_4121, ADD_4122, ADD_4123, 0.119311)){ return false; } } // (1000, 1000)
  auto MUL_1511 = SUB_1478 * 0.7071068;
  auto MUL_1527 = ADD_1473 * 0.7071068;
  auto MUL_1551 = ADD_1473 * 0.36;
  auto MUL_1561 = SUB_1478 * MUL_1551;
  auto MUL_1524 = ADD_1466 * 0.7071068;
  auto SUB_1528 = MUL_1527 - MUL_1524;
  auto ADD_1538 = MUL_1524 + MUL_1527;
  auto MUL_1513 = SUB_1459 * 0.7071068;
  auto SUB_1543 = MUL_1511 - MUL_1513;
  auto ADD_1514 = MUL_1511 + MUL_1513;
  auto MUL_1554 = SUB_1459 * 0.36;
  auto MUL_1563 = ADD_1466 * MUL_1554;
  auto ADD_1565 = MUL_1561 + MUL_1563;
  auto MUL_1569 = ADD_1565 * 2.0;
  auto SUB_1592 = ADD_1283 - MUL_1569;
  auto INPUT_11 = q[11];
  auto DIV_1596 = INPUT_11 * 0.5;
  auto SIN_1597 = DIV_1596.sin();
  auto COS_1603 = DIV_1596.cos();
  auto MUL_1620 = SUB_1543 * COS_1603;
  auto MUL_1615 = SUB_1543 * SIN_1597;
  auto MUL_1618 = ADD_1538 * COS_1603;
  auto ADD_1619 = MUL_1615 + MUL_1618;
  auto MUL_4131 = ADD_1619 * ADD_1619;
  auto MUL_1623 = ADD_1538 * SIN_1597;
  auto SUB_1624 = MUL_1620 - MUL_1623;
  auto MUL_4132 = SUB_1624 * ADD_1619;
  auto MUL_1605 = ADD_1514 * COS_1603;
  auto MUL_1610 = ADD_1514 * SIN_1597;
  auto MUL_1612 = SUB_1528 * COS_1603;
  auto SUB_1613 = MUL_1612 - MUL_1610;
  auto MUL_4133 = SUB_1624 * SUB_1613;
  auto MUL_4130 = SUB_1613 * SUB_1613;
  auto ADD_4139 = MUL_4130 + MUL_4131;
  auto MUL_1606 = SUB_1528 * SIN_1597;
  auto ADD_1607 = MUL_1605 + MUL_1606;
  auto MUL_4137 = ADD_1607 * ADD_1619;
  auto ADD_4165 = MUL_4137 + MUL_4133;
  auto MUL_4136 = ADD_1607 * SUB_1613;
  auto SUB_4152 = MUL_4136 - MUL_4132;
  auto MUL_4167 = ADD_4165 * 2.0;
  auto MUL_4204 = MUL_4167 * 0.0299598;
  auto MUL_4154 = SUB_4152 * 2.0;
  auto MUL_4192 = MUL_4154 * 1.19e-05;
  auto MUL_4142 = ADD_4139 * 2.0;
  auto SUB_4145 = 1.0 - MUL_4142;
  auto MUL_4180 = SUB_4145 * 0.0006389;
  auto ADD_4214 = MUL_4180 + MUL_4192;
  auto ADD_4220 = ADD_4214 + MUL_4204;
  auto SUB_4226 = SUB_1592 - ADD_4220;
  auto ADD_4146 = MUL_4136 + MUL_4132;
  auto MUL_4135 = SUB_1624 * ADD_1607;
  auto MUL_4138 = SUB_1613 * ADD_1619;
  auto SUB_4168 = MUL_4138 - MUL_4135;
  auto MUL_4134 = ADD_1607 * ADD_1607;
  auto ADD_4155 = MUL_4131 + MUL_4134;
  auto MUL_1576 = ADD_1473 * MUL_1551;
  auto MUL_1574 = SUB_1459 * MUL_1554;
  auto ADD_1578 = MUL_1574 + MUL_1576;
  auto MUL_1581 = ADD_1578 * 2.0;
  auto SUB_1584 = 0.36 - MUL_1581;
  auto ADD_1593 = SUB_1293 + SUB_1584;
  auto MUL_4170 = SUB_4168 * 2.0;
  auto MUL_4208 = MUL_4170 * 0.0299598;
  auto MUL_4158 = ADD_4155 * 2.0;
  auto SUB_4161 = 1.0 - MUL_4158;
  auto MUL_4196 = SUB_4161 * 1.19e-05;
  auto MUL_4148 = ADD_4146 * 2.0;
  auto MUL_4184 = MUL_4148 * 0.0006389;
  auto ADD_4216 = MUL_4184 + MUL_4196;
  auto ADD_4222 = ADD_4216 + MUL_4208;
  auto SUB_4227 = ADD_1593 - ADD_4222;
  auto SUB_4149 = MUL_4137 - MUL_4133;
  auto ADD_4162 = MUL_4138 + MUL_4135;
  auto ADD_4171 = MUL_4130 + MUL_4134;
  auto MUL_1585 = SUB_1478 * MUL_1554;
  auto MUL_1587 = ADD_1466 * MUL_1551;
  auto SUB_1588 = MUL_1585 - MUL_1587;
  auto MUL_1590 = SUB_1588 * 2.0;
  auto ADD_1594 = 1.2 + MUL_1590;
  auto MUL_4174 = ADD_4171 * 2.0;
  auto SUB_4177 = 1.0 - MUL_4174;
  auto MUL_4212 = SUB_4177 * 0.0299598;
  auto MUL_4164 = ADD_4162 * 2.0;
  auto MUL_4200 = MUL_4164 * 1.19e-05;
  auto MUL_4151 = SUB_4149 * 2.0;
  auto MUL_4188 = MUL_4151 * 0.0006389;
  auto ADD_4218 = MUL_4188 + MUL_4200;
  auto ADD_4224 = ADD_4218 + MUL_4212;
  auto SUB_4228 = ADD_1594 - ADD_4224;
  auto MUL_4242 = MUL_4167 * 0.116149;
  auto MUL_4236 = MUL_4154 * 0.0035861;
  auto MUL_4230 = SUB_4145 * 0.0006352;
  auto ADD_4247 = MUL_4230 + MUL_4236;
  auto ADD_4250 = ADD_4247 + MUL_4242;
  auto ADD_4253 = SUB_1592 + ADD_4250;
  auto MUL_4244 = MUL_4170 * 0.116149;
  auto MUL_4238 = SUB_4161 * 0.0035861;
  auto MUL_4232 = MUL_4148 * 0.0006352;
  auto ADD_4248 = MUL_4232 + MUL_4238;
  auto ADD_4251 = ADD_4248 + MUL_4244;
  auto ADD_4254 = ADD_1593 + ADD_4251;
  auto MUL_4246 = SUB_4177 * 0.116149;
  auto MUL_4240 = MUL_4164 * 0.0035861;
  auto MUL_4234 = MUL_4151 * 0.0006352;
  auto ADD_4249 = MUL_4234 + MUL_4240;
  auto ADD_4252 = ADD_4249 + MUL_4246;
  auto ADD_4255 = ADD_1594 + ADD_4252;
  auto MUL_4275 = MUL_4167 * 0.011294;
  auto MUL_4269 = MUL_4154 * 0.0184731;
  auto MUL_4258 = SUB_4145 * 0.0006778;
  auto SUB_4280 = MUL_4269 - MUL_4258;
  auto ADD_4283 = SUB_4280 + MUL_4275;
  auto ADD_4286 = SUB_1592 + ADD_4283;
  auto MUL_4277 = MUL_4170 * 0.011294;
  auto MUL_4271 = SUB_4161 * 0.0184731;
  auto MUL_4262 = MUL_4148 * 0.0006778;
  auto SUB_4281 = MUL_4271 - MUL_4262;
  auto ADD_4284 = SUB_4281 + MUL_4277;
  auto ADD_4287 = ADD_1593 + ADD_4284;
  auto MUL_4279 = SUB_4177 * 0.011294;
  auto MUL_4273 = MUL_4164 * 0.0184731;
  auto MUL_4266 = MUL_4151 * 0.0006778;
  auto SUB_4282 = MUL_4273 - MUL_4266;
  auto ADD_4285 = SUB_4282 + MUL_4279;
  auto ADD_4288 = ADD_1594 + ADD_4285;
  if(/*arm_left_link_5_r vs. arm_right_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3290, ADD_3291, ADD_3292, 0.095701, SUB_4226, SUB_4227, SUB_4228, 0.080043)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; } } // (1000, 1126)
  if(/*arm_left_link_6_b vs. arm_right_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3446, ADD_3447, ADD_3448, 0.119311, SUB_4226, SUB_4227, SUB_4228, 0.080043)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; } } // (1126, 1126)
  if(/*arm_left_link_7_t vs. arm_right_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3593, ADD_3594, ADD_3595, 0.095372, SUB_4226, SUB_4227, SUB_4228, 0.080043)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; } } // (1126, 1126)
  if(/*arm_right_link_3_e*/ sphere_environment_in_collision(environment, SUB_4226, SUB_4227, SUB_4228, 0.080043)){ if(sphere_environment_in_collision(environment, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_environment_in_collision(environment, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; } } // (1126, 1126)
  if(/*torso_base_link vs. arm_right_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, SUB_4226, SUB_4227, SUB_4228, 0.080043)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; } } // (1126, 1126)
  if(/*torso_link_b1 vs. arm_right_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_2453, NEGATE_2455, 0.6608069, 0.273417, SUB_4226, SUB_4227, SUB_4228, 0.080043)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; } } // (1126, 1126)
  auto MUL_1658 = SUB_1624 * 0.7071068;
  auto MUL_1675 = ADD_1619 * 0.7071068;
  auto MUL_1672 = SUB_1613 * 0.7071068;
  auto SUB_1677 = MUL_1672 - MUL_1675;
  auto ADD_1688 = MUL_1672 + MUL_1675;
  auto MUL_1661 = ADD_1607 * 0.7071068;
  auto SUB_1662 = MUL_1661 - MUL_1658;
  auto ADD_1695 = MUL_1658 + MUL_1661;
  auto INPUT_12 = q[12];
  auto DIV_1732 = INPUT_12 * 0.5;
  auto SIN_1733 = DIV_1732.sin();
  auto COS_1742 = DIV_1732.cos();
  auto MUL_1740 = SIN_1733 * 1.0;
  auto MUL_1755 = ADD_1695 * MUL_1740;
  auto MUL_1761 = ADD_1695 * COS_1742;
  auto MUL_1764 = ADD_1688 * MUL_1740;
  auto ADD_1766 = MUL_1761 + MUL_1764;
  auto MUL_1759 = ADD_1688 * COS_1742;
  auto SUB_1760 = MUL_1759 - MUL_1755;
  auto MUL_4297 = ADD_1766 * SUB_1760;
  auto MUL_4296 = SUB_1760 * SUB_1760;
  auto MUL_1750 = SUB_1662 * MUL_1740;
  auto MUL_1744 = SUB_1662 * COS_1742;
  auto MUL_1745 = SUB_1677 * MUL_1740;
  auto SUB_1747 = MUL_1744 - MUL_1745;
  auto MUL_4302 = SUB_1747 * SUB_1760;
  auto MUL_1752 = SUB_1677 * COS_1742;
  auto ADD_1753 = MUL_1750 + MUL_1752;
  auto MUL_4298 = ADD_1766 * ADD_1753;
  auto ADD_4330 = MUL_4302 + MUL_4298;
  auto MUL_4301 = SUB_1747 * ADD_1753;
  auto SUB_4317 = MUL_4301 - MUL_4297;
  auto MUL_4295 = ADD_1753 * ADD_1753;
  auto ADD_4304 = MUL_4295 + MUL_4296;
  auto MUL_4332 = ADD_4330 * 2.0;
  auto MUL_4362 = MUL_4332 * 0.1007051;
  auto MUL_4319 = SUB_4317 * 2.0;
  auto MUL_4351 = MUL_4319 * 0.0017953;
  auto MUL_4307 = ADD_4304 * 2.0;
  auto SUB_4310 = 1.0 - MUL_4307;
  auto MUL_4344 = SUB_4310 * 0.0003948;
  auto SUB_4367 = MUL_4344 - MUL_4351;
  auto ADD_4370 = SUB_4367 + MUL_4362;
  auto ADD_4373 = SUB_1592 + ADD_4370;
  auto ADD_4311 = MUL_4301 + MUL_4297;
  auto MUL_4300 = ADD_1766 * SUB_1747;
  auto MUL_4299 = SUB_1747 * SUB_1747;
  auto ADD_4320 = MUL_4296 + MUL_4299;
  auto MUL_4303 = ADD_1753 * SUB_1760;
  auto SUB_4333 = MUL_4303 - MUL_4300;
  auto MUL_4335 = SUB_4333 * 2.0;
  auto MUL_4364 = MUL_4335 * 0.1007051;
  auto MUL_4323 = ADD_4320 * 2.0;
  auto SUB_4326 = 1.0 - MUL_4323;
  auto MUL_4355 = SUB_4326 * 0.0017953;
  auto MUL_4313 = ADD_4311 * 2.0;
  auto MUL_4346 = MUL_4313 * 0.0003948;
  auto SUB_4368 = MUL_4346 - MUL_4355;
  auto ADD_4371 = SUB_4368 + MUL_4364;
  auto ADD_4374 = ADD_1593 + ADD_4371;
  auto SUB_4314 = MUL_4302 - MUL_4298;
  auto ADD_4327 = MUL_4303 + MUL_4300;
  auto ADD_4336 = MUL_4295 + MUL_4299;
  auto MUL_4339 = ADD_4336 * 2.0;
  auto SUB_4342 = 1.0 - MUL_4339;
  auto MUL_4366 = SUB_4342 * 0.1007051;
  auto MUL_4329 = ADD_4327 * 2.0;
  auto MUL_4359 = MUL_4329 * 0.0017953;
  auto MUL_4316 = SUB_4314 * 2.0;
  auto MUL_4348 = MUL_4316 * 0.0003948;
  auto SUB_4369 = MUL_4348 - MUL_4359;
  auto ADD_4372 = SUB_4369 + MUL_4366;
  auto ADD_4375 = ADD_1594 + ADD_4372;
  auto MUL_4396 = MUL_4332 * 0.0051908;
  auto MUL_4389 = MUL_4319 * 0.1581211;
  auto MUL_4378 = SUB_4310 * 0.0002929;
  auto SUB_4406 = MUL_4389 - MUL_4378;
  auto SUB_4409 = SUB_4406 - MUL_4396;
  auto ADD_4412 = SUB_1592 + SUB_4409;
  auto MUL_4400 = MUL_4335 * 0.0051908;
  auto MUL_4391 = SUB_4326 * 0.1581211;
  auto MUL_4382 = MUL_4313 * 0.0002929;
  auto SUB_4407 = MUL_4391 - MUL_4382;
  auto SUB_4410 = SUB_4407 - MUL_4400;
  auto ADD_4413 = ADD_1593 + SUB_4410;
  auto MUL_4404 = SUB_4342 * 0.0051908;
  auto MUL_4393 = MUL_4329 * 0.1581211;
  auto MUL_4386 = MUL_4316 * 0.0002929;
  auto SUB_4408 = MUL_4393 - MUL_4386;
  auto SUB_4411 = SUB_4408 - MUL_4404;
  auto ADD_4414 = ADD_1594 + SUB_4411;
  auto MUL_4435 = MUL_4332 * 0.0213048;
  auto MUL_4428 = MUL_4319 * 0.0524171;
  auto MUL_4417 = SUB_4310 * 0.0014369;
  auto SUB_4445 = MUL_4428 - MUL_4417;
  auto SUB_4448 = SUB_4445 - MUL_4435;
  auto ADD_4451 = SUB_1592 + SUB_4448;
  auto MUL_4439 = MUL_4335 * 0.0213048;
  auto MUL_4430 = SUB_4326 * 0.0524171;
  auto MUL_4421 = MUL_4313 * 0.0014369;
  auto SUB_4446 = MUL_4430 - MUL_4421;
  auto SUB_4449 = SUB_4446 - MUL_4439;
  auto ADD_4452 = ADD_1593 + SUB_4449;
  auto MUL_4443 = SUB_4342 * 0.0213048;
  auto MUL_4432 = MUL_4329 * 0.0524171;
  auto MUL_4425 = MUL_4316 * 0.0014369;
  auto SUB_4447 = MUL_4432 - MUL_4425;
  auto SUB_4450 = SUB_4447 - MUL_4443;
  auto ADD_4453 = ADD_1594 + SUB_4450;
  auto MUL_4480 = MUL_4332 * 0.0299598;
  auto MUL_4468 = MUL_4319 * 1.19e-05;
  auto MUL_4456 = SUB_4310 * 0.0006389;
  auto ADD_4490 = MUL_4456 + MUL_4468;
  auto ADD_4496 = ADD_4490 + MUL_4480;
  auto SUB_4502 = SUB_1592 - ADD_4496;
  auto MUL_4484 = MUL_4335 * 0.0299598;
  auto MUL_4472 = SUB_4326 * 1.19e-05;
  auto MUL_4460 = MUL_4313 * 0.0006389;
  auto ADD_4492 = MUL_4460 + MUL_4472;
  auto ADD_4498 = ADD_4492 + MUL_4484;
  auto SUB_4503 = ADD_1593 - ADD_4498;
  auto MUL_4488 = SUB_4342 * 0.0299598;
  auto MUL_4476 = MUL_4329 * 1.19e-05;
  auto MUL_4464 = MUL_4316 * 0.0006389;
  auto ADD_4494 = MUL_4464 + MUL_4476;
  auto ADD_4500 = ADD_4494 + MUL_4488;
  auto SUB_4504 = ADD_1594 - ADD_4500;
  if(/*arm_left_link_4_u vs. arm_right_link_4_u*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3072, ADD_3073, ADD_3074, 0.153255, ADD_4373, ADD_4374, ADD_4375, 0.079093)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3111, ADD_3112, ADD_3113, 0.089005, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3111, ADD_3112, ADD_3113, 0.089005, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3111, ADD_3112, ADD_3113, 0.089005, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3150, ADD_3151, ADD_3152, 0.088818, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3150, ADD_3151, ADD_3152, 0.088818, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3150, ADD_3151, ADD_3152, 0.088818, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3201, SUB_3202, SUB_3203, 0.080043, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3201, SUB_3202, SUB_3203, 0.080043, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3201, SUB_3202, SUB_3203, 0.080043, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; } } // (1126, 1253)
  if(/*arm_left_link_5_r vs. arm_right_link_4_u*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3290, ADD_3291, ADD_3292, 0.095701, ADD_4373, ADD_4374, ADD_4375, 0.079093)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; } } // (1253, 1253)
  if(/*arm_left_link_6_b vs. arm_right_link_4_u*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3446, ADD_3447, ADD_3448, 0.119311, ADD_4373, ADD_4374, ADD_4375, 0.079093)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; } } // (1253, 1253)
  if(/*arm_left_link_7_t vs. arm_right_link_4_u*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3593, ADD_3594, ADD_3595, 0.095372, ADD_4373, ADD_4374, ADD_4375, 0.079093)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; } } // (1253, 1253)
  if(/*arm_right_link_2_l vs. arm_right_link_4_u*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_4056, ADD_4057, ADD_4058, 0.088818, ADD_4373, ADD_4374, ADD_4375, 0.079093)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4092, ADD_4093, ADD_4094, 0.095701, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4092, ADD_4093, ADD_4094, 0.095701, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4092, ADD_4093, ADD_4094, 0.095701, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4121, ADD_4122, ADD_4123, 0.119311, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4121, ADD_4122, ADD_4123, 0.119311, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4121, ADD_4122, ADD_4123, 0.119311, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; } } // (1253, 1253)
  if(/*arm_right_link_4_u*/ sphere_environment_in_collision(environment, ADD_4373, ADD_4374, ADD_4375, 0.079093)){ if(sphere_environment_in_collision(environment, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_environment_in_collision(environment, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_environment_in_collision(environment, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; } } // (1253, 1253)
  if(/*torso_base_link vs. arm_right_link_4_u*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4373, ADD_4374, ADD_4375, 0.079093)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; } } // (1253, 1253)
  if(/*torso_link_b1 vs. arm_right_link_4_u*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_2453, NEGATE_2455, 0.6608069, 0.273417, ADD_4373, ADD_4374, ADD_4375, 0.079093)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_4412, ADD_4413, ADD_4414, 0.089005)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_4451, ADD_4452, ADD_4453, 0.088818)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, SUB_4502, SUB_4503, SUB_4504, 0.080043)){ return false; } } // (1253, 1253)
  auto MUL_1827 = ADD_1766 * 0.7071068;
  auto MUL_1824 = SUB_1760 * 0.7071068;
  auto MUL_1838 = SUB_1760 * 0.36;
  auto MUL_1848 = ADD_1766 * MUL_1838;
  auto MUL_1829 = SUB_1747 * 0.7071068;
  auto SUB_1830 = MUL_1827 - MUL_1829;
  auto ADD_1802 = MUL_1827 + MUL_1829;
  auto MUL_1841 = SUB_1747 * 0.36;
  auto MUL_1850 = ADD_1753 * MUL_1841;
  auto SUB_1851 = MUL_1850 - MUL_1848;
  auto MUL_1854 = SUB_1851 * 2.0;
  auto ADD_1877 = SUB_1592 + MUL_1854;
  auto MUL_1821 = ADD_1753 * 0.7071068;
  auto SUB_1825 = MUL_1824 - MUL_1821;
  auto ADD_1815 = MUL_1821 + MUL_1824;
  auto INPUT_13 = q[13];
  auto DIV_1881 = INPUT_13 * 0.5;
  auto SIN_1882 = DIV_1881.sin();
  auto COS_1891 = DIV_1881.cos();
  auto MUL_1889 = SIN_1882 * 1.0;
  auto MUL_1904 = SUB_1830 * MUL_1889;
  auto MUL_1910 = SUB_1830 * COS_1891;
  auto MUL_1913 = SUB_1825 * MUL_1889;
  auto ADD_1915 = MUL_1910 + MUL_1913;
  auto MUL_1908 = SUB_1825 * COS_1891;
  auto SUB_1909 = MUL_1908 - MUL_1904;
  auto MUL_4515 = ADD_1915 * SUB_1909;
  auto MUL_4514 = SUB_1909 * SUB_1909;
  auto MUL_1899 = ADD_1802 * MUL_1889;
  auto MUL_1893 = ADD_1802 * COS_1891;
  auto MUL_1894 = ADD_1815 * MUL_1889;
  auto SUB_1896 = MUL_1893 - MUL_1894;
  auto MUL_4520 = SUB_1896 * SUB_1909;
  auto MUL_1901 = ADD_1815 * COS_1891;
  auto ADD_1902 = MUL_1899 + MUL_1901;
  auto MUL_4516 = ADD_1915 * ADD_1902;
  auto ADD_4548 = MUL_4520 + MUL_4516;
  auto MUL_4519 = SUB_1896 * ADD_1902;
  auto SUB_4535 = MUL_4519 - MUL_4515;
  auto MUL_4513 = ADD_1902 * ADD_1902;
  auto ADD_4522 = MUL_4513 + MUL_4514;
  auto MUL_4550 = ADD_4548 * 2.0;
  auto MUL_4586 = MUL_4550 * 0.0120211;
  auto MUL_4537 = SUB_4535 * 2.0;
  auto MUL_4575 = MUL_4537 * 0.0169963;
  auto MUL_4525 = ADD_4522 * 2.0;
  auto SUB_4528 = 1.0 - MUL_4525;
  auto MUL_4563 = SUB_4528 * 0.0010222;
  auto ADD_4591 = MUL_4563 + MUL_4575;
  auto SUB_4597 = MUL_4586 - ADD_4591;
  auto ADD_4600 = ADD_1877 + SUB_4597;
  auto ADD_4529 = MUL_4519 + MUL_4515;
  auto MUL_4518 = ADD_1915 * SUB_1896;
  auto MUL_4517 = SUB_1896 * SUB_1896;
  auto ADD_4538 = MUL_4514 + MUL_4517;
  auto MUL_4521 = ADD_1902 * SUB_1909;
  auto SUB_4551 = MUL_4521 - MUL_4518;
  auto MUL_1860 = SUB_1760 * MUL_1838;
  auto MUL_1858 = SUB_1747 * MUL_1841;
  auto ADD_1862 = MUL_1858 + MUL_1860;
  auto MUL_1865 = ADD_1862 * 2.0;
  auto SUB_1868 = 0.36 - MUL_1865;
  auto ADD_1878 = ADD_1593 + SUB_1868;
  auto MUL_4553 = SUB_4551 * 2.0;
  auto MUL_4588 = MUL_4553 * 0.0120211;
  auto MUL_4541 = ADD_4538 * 2.0;
  auto SUB_4544 = 1.0 - MUL_4541;
  auto MUL_4579 = SUB_4544 * 0.0169963;
  auto MUL_4531 = ADD_4529 * 2.0;
  auto MUL_4567 = MUL_4531 * 0.0010222;
  auto ADD_4593 = MUL_4567 + MUL_4579;
  auto SUB_4598 = MUL_4588 - ADD_4593;
  auto ADD_4601 = ADD_1878 + SUB_4598;
  auto SUB_4532 = MUL_4520 - MUL_4516;
  auto ADD_4545 = MUL_4521 + MUL_4518;
  auto ADD_4554 = MUL_4513 + MUL_4517;
  auto MUL_1869 = ADD_1766 * MUL_1841;
  auto MUL_1871 = ADD_1753 * MUL_1838;
  auto ADD_1873 = MUL_1869 + MUL_1871;
  auto MUL_1875 = ADD_1873 * 2.0;
  auto ADD_1879 = ADD_1594 + MUL_1875;
  auto MUL_4557 = ADD_4554 * 2.0;
  auto SUB_4560 = 1.0 - MUL_4557;
  auto MUL_4590 = SUB_4560 * 0.0120211;
  auto MUL_4547 = ADD_4545 * 2.0;
  auto MUL_4583 = MUL_4547 * 0.0169963;
  auto MUL_4534 = SUB_4532 * 2.0;
  auto MUL_4571 = MUL_4534 * 0.0010222;
  auto ADD_4595 = MUL_4571 + MUL_4583;
  auto SUB_4599 = MUL_4590 - ADD_4595;
  auto ADD_4602 = ADD_1879 + SUB_4599;
  auto MUL_4622 = MUL_4550 * 0.1007051;
  auto MUL_4611 = MUL_4537 * 0.0017953;
  auto MUL_4604 = SUB_4528 * 0.0003948;
  auto SUB_4627 = MUL_4604 - MUL_4611;
  auto ADD_4630 = SUB_4627 + MUL_4622;
  auto ADD_4633 = ADD_1877 + ADD_4630;
  auto MUL_4624 = MUL_4553 * 0.1007051;
  auto MUL_4615 = SUB_4544 * 0.0017953;
  auto MUL_4606 = MUL_4531 * 0.0003948;
  auto SUB_4628 = MUL_4606 - MUL_4615;
  auto ADD_4631 = SUB_4628 + MUL_4624;
  auto ADD_4634 = ADD_1878 + ADD_4631;
  auto MUL_4626 = SUB_4560 * 0.1007051;
  auto MUL_4619 = MUL_4547 * 0.0017953;
  auto MUL_4608 = MUL_4534 * 0.0003948;
  auto SUB_4629 = MUL_4608 - MUL_4619;
  auto ADD_4632 = SUB_4629 + MUL_4626;
  auto ADD_4635 = ADD_1879 + ADD_4632;
  if(/*arm_left_link_3_e vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2945, ADD_2946, ADD_2947, 0.095372, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; } // (1253, 1362)
  if(/*arm_left_link_3_e vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2980, ADD_2981, ADD_2982, 0.085433, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; } // (1362, 1362)
  auto SUB_4672 = MUL_4586 - ADD_4591;
  auto ADD_4675 = ADD_1877 + SUB_4672;
  auto SUB_4673 = MUL_4588 - ADD_4593;
  auto ADD_4676 = ADD_1878 + SUB_4673;
  auto SUB_4674 = MUL_4590 - ADD_4595;
  auto ADD_4677 = ADD_1879 + SUB_4674;
  if(/*arm_left_link_2_l vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2804, ADD_2805, ADD_2806, 0.187648, ADD_4600, ADD_4601, ADD_4602, 0.077046)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } } // (1362, 1368)
  if(/*arm_left_link_3_e vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2945, ADD_2946, ADD_2947, 0.095372, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } // (1368, 1368)
  if(/*arm_left_link_3_e vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2980, ADD_2981, ADD_2982, 0.085433, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } // (1368, 1368)
  if(/*arm_left_link_4_u vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3072, ADD_3073, ADD_3074, 0.153255, ADD_4600, ADD_4601, ADD_4602, 0.077046)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3111, ADD_3112, ADD_3113, 0.089005, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3111, ADD_3112, ADD_3113, 0.089005, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3150, ADD_3151, ADD_3152, 0.088818, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3150, ADD_3151, ADD_3152, 0.088818, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3201, SUB_3202, SUB_3203, 0.080043, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3201, SUB_3202, SUB_3203, 0.080043, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } } // (1368, 1368)
  if(/*arm_left_link_5_r vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3290, ADD_3291, ADD_3292, 0.095701, ADD_4600, ADD_4601, ADD_4602, 0.077046)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } } // (1368, 1368)
  if(/*arm_left_link_6_b vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3446, ADD_3447, ADD_3448, 0.119311, ADD_4600, ADD_4601, ADD_4602, 0.077046)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } } // (1368, 1368)
  if(/*arm_left_link_7_t vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3593, ADD_3594, ADD_3595, 0.095372, ADD_4600, ADD_4601, ADD_4602, 0.077046)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } } // (1368, 1368)
  if(/*arm_right_link_2_l vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_4056, ADD_4057, ADD_4058, 0.088818, ADD_4600, ADD_4601, ADD_4602, 0.077046)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4092, ADD_4093, ADD_4094, 0.095701, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4092, ADD_4093, ADD_4094, 0.095701, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4121, ADD_4122, ADD_4123, 0.119311, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4121, ADD_4122, ADD_4123, 0.119311, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } } // (1368, 1368)
  if(/*arm_right_link_5_r*/ sphere_environment_in_collision(environment, ADD_4600, ADD_4601, ADD_4602, 0.077046)){ if(sphere_environment_in_collision(environment, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_environment_in_collision(environment, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } } // (1368, 1368)
  if(/*torso_base_link vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4600, ADD_4601, ADD_4602, 0.077046)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } } // (1368, 1368)
  if(/*torso_link_b1 vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_2453, NEGATE_2455, 0.6608069, 0.273417, ADD_4600, ADD_4601, ADD_4602, 0.077046)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } } // (1368, 1368)
  auto MUL_1949 = ADD_1915 * 0.7071068;
  auto MUL_1966 = SUB_1909 * 0.7071068;
  auto MUL_1952 = SUB_1896 * 0.7071068;
  auto SUB_1953 = MUL_1952 - MUL_1949;
  auto ADD_1986 = MUL_1949 + MUL_1952;
  auto MUL_1963 = ADD_1902 * 0.7071068;
  auto SUB_1968 = MUL_1963 - MUL_1966;
  auto ADD_1979 = MUL_1963 + MUL_1966;
  auto INPUT_14 = q[14];
  auto DIV_2023 = INPUT_14 * 0.5;
  auto SIN_2024 = DIV_2023.sin();
  auto COS_2033 = DIV_2023.cos();
  auto MUL_2031 = SIN_2024 * 1.0;
  auto MUL_2046 = ADD_1986 * MUL_2031;
  auto MUL_2052 = ADD_1986 * COS_2033;
  auto MUL_2055 = ADD_1979 * MUL_2031;
  auto ADD_2057 = MUL_2052 + MUL_2055;
  auto MUL_2050 = ADD_1979 * COS_2033;
  auto SUB_2051 = MUL_2050 - MUL_2046;
  auto MUL_4686 = ADD_2057 * SUB_2051;
  auto MUL_4685 = SUB_2051 * SUB_2051;
  auto MUL_2041 = SUB_1953 * MUL_2031;
  auto MUL_2035 = SUB_1953 * COS_2033;
  auto MUL_2036 = SUB_1968 * MUL_2031;
  auto SUB_2038 = MUL_2035 - MUL_2036;
  auto MUL_4691 = SUB_2038 * SUB_2051;
  auto MUL_2043 = SUB_1968 * COS_2033;
  auto ADD_2044 = MUL_2041 + MUL_2043;
  auto MUL_4687 = ADD_2057 * ADD_2044;
  auto ADD_4719 = MUL_4691 + MUL_4687;
  auto MUL_4690 = SUB_2038 * ADD_2044;
  auto SUB_4706 = MUL_4690 - MUL_4686;
  auto MUL_4684 = ADD_2044 * ADD_2044;
  auto ADD_4693 = MUL_4684 + MUL_4685;
  auto MUL_4721 = ADD_4719 * 2.0;
  auto MUL_4751 = MUL_4721 * 0.0063689;
  auto MUL_4708 = SUB_4706 * 2.0;
  auto MUL_4745 = MUL_4708 * 0.0819007;
  auto MUL_4696 = ADD_4693 * 2.0;
  auto SUB_4699 = 1.0 - MUL_4696;
  auto MUL_4734 = SUB_4699 * 1.14e-05;
  auto SUB_4756 = MUL_4745 - MUL_4734;
  auto ADD_4759 = SUB_4756 + MUL_4751;
  auto ADD_4762 = ADD_1877 + ADD_4759;
  auto ADD_4700 = MUL_4690 + MUL_4686;
  auto MUL_4689 = ADD_2057 * SUB_2038;
  auto MUL_4688 = SUB_2038 * SUB_2038;
  auto ADD_4709 = MUL_4685 + MUL_4688;
  auto MUL_4692 = ADD_2044 * SUB_2051;
  auto SUB_4722 = MUL_4692 - MUL_4689;
  auto MUL_4724 = SUB_4722 * 2.0;
  auto MUL_4753 = MUL_4724 * 0.0063689;
  auto MUL_4712 = ADD_4709 * 2.0;
  auto SUB_4715 = 1.0 - MUL_4712;
  auto MUL_4747 = SUB_4715 * 0.0819007;
  auto MUL_4702 = ADD_4700 * 2.0;
  auto MUL_4738 = MUL_4702 * 1.14e-05;
  auto SUB_4757 = MUL_4747 - MUL_4738;
  auto ADD_4760 = SUB_4757 + MUL_4753;
  auto ADD_4763 = ADD_1878 + ADD_4760;
  auto SUB_4703 = MUL_4691 - MUL_4687;
  auto ADD_4716 = MUL_4692 + MUL_4689;
  auto ADD_4725 = MUL_4684 + MUL_4688;
  auto MUL_4728 = ADD_4725 * 2.0;
  auto SUB_4731 = 1.0 - MUL_4728;
  auto MUL_4755 = SUB_4731 * 0.0063689;
  auto MUL_4718 = ADD_4716 * 2.0;
  auto MUL_4749 = MUL_4718 * 0.0819007;
  auto MUL_4705 = SUB_4703 * 2.0;
  auto MUL_4742 = MUL_4705 * 1.14e-05;
  auto SUB_4758 = MUL_4749 - MUL_4742;
  auto ADD_4761 = SUB_4758 + MUL_4755;
  auto ADD_4764 = ADD_1879 + ADD_4761;
  auto ADD_4792 = SUB_4756 + MUL_4751;
  auto ADD_4795 = ADD_1877 + ADD_4792;
  auto ADD_4793 = SUB_4757 + MUL_4753;
  auto ADD_4796 = ADD_1878 + ADD_4793;
  auto ADD_4794 = SUB_4758 + MUL_4755;
  auto ADD_4797 = ADD_1879 + ADD_4794;
  if(/*arm_left_link_3_e vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2945, ADD_2946, ADD_2947, 0.095372, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; } // (1368, 1447)
  if(/*arm_left_link_3_e vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2980, ADD_2981, ADD_2982, 0.085433, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; } // (1447, 1447)
  auto MUL_4817 = MUL_4721 * 0.0259799;
  auto MUL_4811 = MUL_4708 * 0.0086577;
  auto MUL_4800 = SUB_4699 * 0.0001404;
  auto SUB_4822 = MUL_4811 - MUL_4800;
  auto ADD_4825 = SUB_4822 + MUL_4817;
  auto ADD_4828 = ADD_1877 + ADD_4825;
  auto MUL_4819 = MUL_4724 * 0.0259799;
  auto MUL_4813 = SUB_4715 * 0.0086577;
  auto MUL_4804 = MUL_4702 * 0.0001404;
  auto SUB_4823 = MUL_4813 - MUL_4804;
  auto ADD_4826 = SUB_4823 + MUL_4819;
  auto ADD_4829 = ADD_1878 + ADD_4826;
  auto MUL_4821 = SUB_4731 * 0.0259799;
  auto MUL_4815 = MUL_4718 * 0.0086577;
  auto MUL_4808 = MUL_4705 * 0.0001404;
  auto SUB_4824 = MUL_4815 - MUL_4808;
  auto ADD_4827 = SUB_4824 + MUL_4821;
  auto ADD_4830 = ADD_1879 + ADD_4827;
  if(/*arm_left_link_2_l vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2804, ADD_2805, ADD_2806, 0.187648, ADD_4762, ADD_4763, ADD_4764, 0.077881)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } } // (1447, 1465)
  if(/*arm_left_link_3_e vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2945, ADD_2946, ADD_2947, 0.095372, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } // (1465, 1465)
  if(/*arm_left_link_3_e vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2980, ADD_2981, ADD_2982, 0.085433, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } // (1465, 1465)
  if(/*arm_left_link_4_u vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3072, ADD_3073, ADD_3074, 0.153255, ADD_4762, ADD_4763, ADD_4764, 0.077881)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3111, ADD_3112, ADD_3113, 0.089005, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3111, ADD_3112, ADD_3113, 0.089005, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3150, ADD_3151, ADD_3152, 0.088818, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3150, ADD_3151, ADD_3152, 0.088818, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3201, SUB_3202, SUB_3203, 0.080043, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3201, SUB_3202, SUB_3203, 0.080043, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } } // (1465, 1465)
  if(/*arm_left_link_5_r vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3290, ADD_3291, ADD_3292, 0.095701, ADD_4762, ADD_4763, ADD_4764, 0.077881)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } } // (1465, 1465)
  if(/*arm_left_link_6_b vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3446, ADD_3447, ADD_3448, 0.119311, ADD_4762, ADD_4763, ADD_4764, 0.077881)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } } // (1465, 1465)
  if(/*arm_left_link_7_t vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3593, ADD_3594, ADD_3595, 0.095372, ADD_4762, ADD_4763, ADD_4764, 0.077881)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } } // (1465, 1465)
  if(/*arm_right_link_2_l vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_4056, ADD_4057, ADD_4058, 0.088818, ADD_4762, ADD_4763, ADD_4764, 0.077881)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4092, ADD_4093, ADD_4094, 0.095701, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4092, ADD_4093, ADD_4094, 0.095701, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4121, ADD_4122, ADD_4123, 0.119311, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4121, ADD_4122, ADD_4123, 0.119311, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } } // (1465, 1465)
  if(/*arm_right_link_6_b*/ sphere_environment_in_collision(environment, ADD_4762, ADD_4763, ADD_4764, 0.077881)){ if(sphere_environment_in_collision(environment, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_environment_in_collision(environment, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } } // (1465, 1465)
  if(/*torso_base_link vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4762, ADD_4763, ADD_4764, 0.077881)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } } // (1465, 1465)
  if(/*torso_link_b1 vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_2453, NEGATE_2455, 0.6608069, 0.273417, ADD_4762, ADD_4763, ADD_4764, 0.077881)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } } // (1465, 1465)
  auto MUL_2091 = ADD_2057 * 0.7071068;
  auto MUL_2108 = SUB_2051 * 0.7071068;
  auto MUL_2136 = SUB_2051 * 0.155;
  auto MUL_2146 = ADD_2057 * MUL_2136;
  auto MUL_2094 = SUB_2038 * 0.7071068;
  auto SUB_2095 = MUL_2094 - MUL_2091;
  auto ADD_2128 = MUL_2091 + MUL_2094;
  auto MUL_2139 = SUB_2038 * 0.155;
  auto MUL_2148 = ADD_2044 * MUL_2139;
  auto SUB_2149 = MUL_2148 - MUL_2146;
  auto MUL_2152 = SUB_2149 * 2.0;
  auto ADD_2175 = ADD_1877 + MUL_2152;
  auto MUL_2105 = ADD_2044 * 0.7071068;
  auto SUB_2110 = MUL_2105 - MUL_2108;
  auto ADD_2121 = MUL_2105 + MUL_2108;
  auto INPUT_15 = q[15];
  auto DIV_2179 = INPUT_15 * 0.5;
  auto SIN_2180 = DIV_2179.sin();
  auto COS_2186 = DIV_2179.cos();
  auto MUL_2203 = ADD_2128 * COS_2186;
  auto MUL_2198 = ADD_2128 * SIN_2180;
  auto MUL_2201 = ADD_2121 * COS_2186;
  auto ADD_2202 = MUL_2198 + MUL_2201;
  auto MUL_4838 = ADD_2202 * ADD_2202;
  auto MUL_2206 = ADD_2121 * SIN_2180;
  auto SUB_2207 = MUL_2203 - MUL_2206;
  auto MUL_4839 = SUB_2207 * ADD_2202;
  auto MUL_2188 = SUB_2095 * COS_2186;
  auto MUL_2193 = SUB_2095 * SIN_2180;
  auto MUL_2195 = SUB_2110 * COS_2186;
  auto SUB_2196 = MUL_2195 - MUL_2193;
  auto MUL_4840 = SUB_2207 * SUB_2196;
  auto MUL_4837 = SUB_2196 * SUB_2196;
  auto ADD_4846 = MUL_4837 + MUL_4838;
  auto MUL_2189 = SUB_2110 * SIN_2180;
  auto ADD_2190 = MUL_2188 + MUL_2189;
  auto MUL_4844 = ADD_2190 * ADD_2202;
  auto ADD_4872 = MUL_4844 + MUL_4840;
  auto MUL_4843 = ADD_2190 * SUB_2196;
  auto SUB_4859 = MUL_4843 - MUL_4839;
  auto MUL_4874 = ADD_4872 * 2.0;
  auto MUL_4904 = MUL_4874 * 0.0259799;
  auto MUL_4861 = SUB_4859 * 2.0;
  auto MUL_4898 = MUL_4861 * 0.0086577;
  auto MUL_4849 = ADD_4846 * 2.0;
  auto SUB_4852 = 1.0 - MUL_4849;
  auto MUL_4887 = SUB_4852 * 0.0001404;
  auto SUB_4909 = MUL_4898 - MUL_4887;
  auto ADD_4912 = SUB_4909 + MUL_4904;
  auto ADD_4915 = ADD_2175 + ADD_4912;
  auto ADD_4853 = MUL_4843 + MUL_4839;
  auto MUL_4842 = SUB_2207 * ADD_2190;
  auto MUL_4845 = SUB_2196 * ADD_2202;
  auto SUB_4875 = MUL_4845 - MUL_4842;
  auto MUL_4841 = ADD_2190 * ADD_2190;
  auto ADD_4862 = MUL_4838 + MUL_4841;
  auto MUL_2158 = SUB_2051 * MUL_2136;
  auto MUL_2156 = SUB_2038 * MUL_2139;
  auto ADD_2160 = MUL_2156 + MUL_2158;
  auto MUL_2163 = ADD_2160 * 2.0;
  auto SUB_2166 = 0.155 - MUL_2163;
  auto ADD_2176 = ADD_1878 + SUB_2166;
  auto MUL_4877 = SUB_4875 * 2.0;
  auto MUL_4906 = MUL_4877 * 0.0259799;
  auto MUL_4865 = ADD_4862 * 2.0;
  auto SUB_4868 = 1.0 - MUL_4865;
  auto MUL_4900 = SUB_4868 * 0.0086577;
  auto MUL_4855 = ADD_4853 * 2.0;
  auto MUL_4891 = MUL_4855 * 0.0001404;
  auto SUB_4910 = MUL_4900 - MUL_4891;
  auto ADD_4913 = SUB_4910 + MUL_4906;
  auto ADD_4916 = ADD_2176 + ADD_4913;
  auto SUB_4856 = MUL_4844 - MUL_4840;
  auto ADD_4869 = MUL_4845 + MUL_4842;
  auto ADD_4878 = MUL_4837 + MUL_4841;
  auto MUL_2167 = ADD_2057 * MUL_2139;
  auto MUL_2169 = ADD_2044 * MUL_2136;
  auto ADD_2171 = MUL_2167 + MUL_2169;
  auto MUL_2173 = ADD_2171 * 2.0;
  auto ADD_2177 = ADD_1879 + MUL_2173;
  auto MUL_4881 = ADD_4878 * 2.0;
  auto SUB_4884 = 1.0 - MUL_4881;
  auto MUL_4908 = SUB_4884 * 0.0259799;
  auto MUL_4871 = ADD_4869 * 2.0;
  auto MUL_4902 = MUL_4871 * 0.0086577;
  auto MUL_4858 = SUB_4856 * 2.0;
  auto MUL_4895 = MUL_4858 * 0.0001404;
  auto SUB_4911 = MUL_4902 - MUL_4895;
  auto ADD_4914 = SUB_4911 + MUL_4908;
  auto ADD_4917 = ADD_2177 + ADD_4914;
  auto MUL_4938 = MUL_4874 * 0.00659;
  auto MUL_4931 = MUL_4861 * 0.018019;
  auto MUL_4920 = SUB_4852 * 0.004017;
  auto SUB_4948 = MUL_4931 - MUL_4920;
  auto SUB_4951 = SUB_4948 - MUL_4938;
  auto ADD_4954 = ADD_2175 + SUB_4951;
  auto MUL_4942 = MUL_4877 * 0.00659;
  auto MUL_4933 = SUB_4868 * 0.018019;
  auto MUL_4924 = MUL_4855 * 0.004017;
  auto SUB_4949 = MUL_4933 - MUL_4924;
  auto SUB_4952 = SUB_4949 - MUL_4942;
  auto ADD_4955 = ADD_2176 + SUB_4952;
  auto MUL_4946 = SUB_4884 * 0.00659;
  auto MUL_4935 = MUL_4871 * 0.018019;
  auto MUL_4928 = MUL_4858 * 0.004017;
  auto SUB_4950 = MUL_4935 - MUL_4928;
  auto SUB_4953 = SUB_4950 - MUL_4946;
  auto ADD_4956 = ADD_2177 + SUB_4953;
  if(/*arm_left_link_3_e vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2945, ADD_2946, ADD_2947, 0.095372, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; } // (1465, 1573)
  if(/*arm_left_link_3_e vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2980, ADD_2981, ADD_2982, 0.085433, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; } // (1573, 1573)
  auto MUL_4977 = MUL_4874 * 0.006585;
  auto MUL_4965 = MUL_4861 * 0.005894;
  auto MUL_4958 = SUB_4852 * 0.017613;
  auto SUB_4987 = MUL_4958 - MUL_4965;
  auto SUB_4990 = SUB_4987 - MUL_4977;
  auto ADD_4993 = ADD_2175 + SUB_4990;
  auto MUL_4981 = MUL_4877 * 0.006585;
  auto MUL_4969 = SUB_4868 * 0.005894;
  auto MUL_4960 = MUL_4855 * 0.017613;
  auto SUB_4988 = MUL_4960 - MUL_4969;
  auto SUB_4991 = SUB_4988 - MUL_4981;
  auto ADD_4994 = ADD_2176 + SUB_4991;
  auto MUL_4985 = SUB_4884 * 0.006585;
  auto MUL_4973 = MUL_4871 * 0.005894;
  auto MUL_4962 = MUL_4858 * 0.017613;
  auto SUB_4989 = MUL_4962 - MUL_4973;
  auto SUB_4992 = SUB_4989 - MUL_4985;
  auto ADD_4995 = ADD_2177 + SUB_4992;
  if(/*arm_left_link_3_e vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2945, ADD_2946, ADD_2947, 0.095372, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; } // (1573, 1591)
  if(/*arm_left_link_3_e vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2980, ADD_2981, ADD_2982, 0.085433, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; } // (1591, 1591)
  auto MUL_5022 = MUL_4874 * 0.006545;
  auto MUL_5010 = MUL_4861 * 0.012936;
  auto MUL_4998 = SUB_4852 * 0.01441;
  auto ADD_5032 = MUL_4998 + MUL_5010;
  auto ADD_5038 = ADD_5032 + MUL_5022;
  auto SUB_5044 = ADD_2175 - ADD_5038;
  auto MUL_5026 = MUL_4877 * 0.006545;
  auto MUL_5014 = SUB_4868 * 0.012936;
  auto MUL_5002 = MUL_4855 * 0.01441;
  auto ADD_5034 = MUL_5002 + MUL_5014;
  auto ADD_5040 = ADD_5034 + MUL_5026;
  auto SUB_5045 = ADD_2176 - ADD_5040;
  auto MUL_5030 = SUB_4884 * 0.006545;
  auto MUL_5018 = MUL_4871 * 0.012936;
  auto MUL_5006 = MUL_4858 * 0.01441;
  auto ADD_5036 = MUL_5006 + MUL_5018;
  auto ADD_5042 = ADD_5036 + MUL_5030;
  auto SUB_5046 = ADD_2177 - ADD_5042;
  if(/*arm_left_link_2_l vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2804, ADD_2805, ADD_2806, 0.187648, ADD_4915, ADD_4916, ADD_4917, 0.068019)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2837, ADD_2838, ADD_2839, 0.095701, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2864, ADD_2865, ADD_2866, 0.119311, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } } // (1591, 1609)
  if(/*arm_left_link_3_e vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2945, ADD_2946, ADD_2947, 0.095372, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } // (1609, 1609)
  if(/*arm_left_link_3_e vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2980, ADD_2981, ADD_2982, 0.085433, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } // (1609, 1609)
  if(/*arm_left_link_4_u vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3072, ADD_3073, ADD_3074, 0.153255, ADD_4915, ADD_4916, ADD_4917, 0.068019)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3111, ADD_3112, ADD_3113, 0.089005, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3111, ADD_3112, ADD_3113, 0.089005, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3111, ADD_3112, ADD_3113, 0.089005, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3150, ADD_3151, ADD_3152, 0.088818, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3150, ADD_3151, ADD_3152, 0.088818, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3150, ADD_3151, ADD_3152, 0.088818, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3201, SUB_3202, SUB_3203, 0.080043, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3201, SUB_3202, SUB_3203, 0.080043, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3201, SUB_3202, SUB_3203, 0.080043, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } } // (1609, 1609)
  if(/*arm_left_link_5_r vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3290, ADD_3291, ADD_3292, 0.095701, ADD_4915, ADD_4916, ADD_4917, 0.068019)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } } // (1609, 1609)
  if(/*arm_left_link_6_b vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3446, ADD_3447, ADD_3448, 0.119311, ADD_4915, ADD_4916, ADD_4917, 0.068019)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } } // (1609, 1609)
  if(/*arm_left_link_7_t vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3593, ADD_3594, ADD_3595, 0.095372, ADD_4915, ADD_4916, ADD_4917, 0.068019)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } } // (1609, 1609)
  if(/*arm_right_link_2_l vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_4056, ADD_4057, ADD_4058, 0.088818, ADD_4915, ADD_4916, ADD_4917, 0.068019)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4092, ADD_4093, ADD_4094, 0.095701, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4092, ADD_4093, ADD_4094, 0.095701, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4092, ADD_4093, ADD_4094, 0.095701, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4121, ADD_4122, ADD_4123, 0.119311, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4121, ADD_4122, ADD_4123, 0.119311, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_4121, ADD_4122, ADD_4123, 0.119311, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } } // (1609, 1609)
  if(/*arm_right_link_7_t*/ sphere_environment_in_collision(environment, ADD_4915, ADD_4916, ADD_4917, 0.068019)){ if(sphere_environment_in_collision(environment, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_environment_in_collision(environment, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_environment_in_collision(environment, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } } // (1609, 1609)
  if(/*torso_base_link vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4915, ADD_4916, ADD_4917, 0.068019)){ if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0552689, -1e-06, 0.2044429, 0.344575, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.0262329, -5e-06, 0.6608069, 0.273417, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(-0.1339749, 5e-06, 0.1743459, 0.325854, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } } // (1609, 1609)
  if(/*torso_link_b1 vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_2453, NEGATE_2455, 0.6608069, 0.273417, ADD_4915, ADD_4916, ADD_4917, 0.068019)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2484, SUB_2485, 1.186491, 0.186756, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_2511, ADD_2512, 1.010525, 0.187648, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } } // (1609, 1609)
  auto MUL_2651 = MUL_2619 * 1.4e-05;
  auto MUL_2644 = SUB_2610 * 0.0884418;
  auto SUB_2673 = MUL_2644 - MUL_2651;
  auto MUL_2663 = MUL_2632 * 0.013509;
  auto SUB_2676 = SUB_2673 - MUL_2663;
  auto ADD_2679 = SUB_147 + SUB_2676;
  auto MUL_2667 = MUL_2635 * 0.013509;
  auto MUL_2655 = SUB_2626 * 1.4e-05;
  auto MUL_2646 = MUL_2613 * 0.0884418;
  auto SUB_2674 = MUL_2646 - MUL_2655;
  auto SUB_2677 = SUB_2674 - MUL_2667;
  auto ADD_2680 = ADD_157 + SUB_2677;
  auto MUL_2671 = SUB_2642 * 0.013509;
  auto MUL_2659 = MUL_2629 * 1.4e-05;
  auto MUL_2648 = MUL_2616 * 0.0884418;
  auto SUB_2675 = MUL_2648 - MUL_2659;
  auto SUB_2678 = SUB_2675 - MUL_2671;
  auto ADD_2681 = 1.2 + SUB_2678;
  if(/*arm_left_link_1_s vs. arm_left_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2679, ADD_2680, ADD_2681, 0.186756, ADD_3290, ADD_3291, ADD_3292, 0.095701)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_3323, ADD_3324, ADD_3325, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_3365, ADD_3366, ADD_3367, 0.077046)){ return false; } } // (1609, 1627)
  if(/*arm_left_link_1_s vs. arm_left_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2679, ADD_2680, ADD_2681, 0.186756, ADD_3446, ADD_3447, ADD_3448, 0.119311)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_3479, ADD_3480, ADD_3481, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_3512, ADD_3513, ADD_3514, 0.068019)){ return false; } } // (1627, 1627)
  if(/*arm_left_link_1_s vs. arm_left_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2679, ADD_2680, ADD_2681, 0.186756, ADD_3593, ADD_3594, ADD_3595, 0.095372)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_3632, ADD_3633, ADD_3634, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_3671, ADD_3672, ADD_3673, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, SUB_3722, SUB_3723, SUB_3724, 0.034568)){ return false; } } // (1627, 1627)
  if(/*arm_left_link_1_s vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2679, ADD_2680, ADD_2681, 0.186756, ADD_4600, ADD_4601, ADD_4602, 0.077046)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } } // (1627, 1627)
  if(/*arm_left_link_1_s vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2679, ADD_2680, ADD_2681, 0.186756, ADD_4762, ADD_4763, ADD_4764, 0.077881)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } } // (1627, 1627)
  if(/*arm_left_link_1_s vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_2679, ADD_2680, ADD_2681, 0.186756, ADD_4915, ADD_4916, ADD_4917, 0.068019)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_2721, ADD_2722, ADD_2723, 0.153255, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } } // (1627, 1627)
  auto MUL_3899 = MUL_3865 * 0.0051908;
  auto MUL_3891 = MUL_3851 * 0.1581211;
  auto MUL_3880 = SUB_3841 * 0.0002929;
  auto ADD_3908 = MUL_3880 + MUL_3891;
  auto ADD_3912 = ADD_3908 + MUL_3899;
  auto SUB_3916 = ADD_1283 - ADD_3912;
  auto MUL_3903 = MUL_3869 * 0.0051908;
  auto MUL_3894 = SUB_3859 * 0.1581211;
  auto MUL_3884 = MUL_3844 * 0.0002929;
  auto SUB_3910 = MUL_3894 - MUL_3884;
  auto ADD_3914 = SUB_3910 + MUL_3903;
  auto ADD_3917 = SUB_1293 + ADD_3914;
  auto MUL_3906 = SUB_3877 * 0.0051908;
  auto MUL_3896 = MUL_3862 * 0.1581211;
  auto MUL_3888 = MUL_3847 * 0.0002929;
  auto SUB_3911 = MUL_3896 - MUL_3888;
  auto SUB_3915 = SUB_3911 - MUL_3906;
  auto ADD_3918 = 1.2 + SUB_3915;
  if(/*arm_left_link_5_r vs. arm_right_link_1_s*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3290, ADD_3291, ADD_3292, 0.095701, SUB_3916, ADD_3917, ADD_3918, 0.089005)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3323, ADD_3324, ADD_3325, 0.079093, ADD_3958, SUB_3959, ADD_3960, 0.153255)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3365, ADD_3366, ADD_3367, 0.077046, ADD_3958, SUB_3959, ADD_3960, 0.153255)){ return false; } } // (1627, 1645)
  if(/*arm_left_link_6_b vs. arm_right_link_1_s*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3446, ADD_3447, ADD_3448, 0.119311, SUB_3916, ADD_3917, ADD_3918, 0.089005)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3479, ADD_3480, ADD_3481, 0.077881, ADD_3958, SUB_3959, ADD_3960, 0.153255)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3512, ADD_3513, ADD_3514, 0.068019, ADD_3958, SUB_3959, ADD_3960, 0.153255)){ return false; } } // (1645, 1645)
  if(/*arm_left_link_7_t vs. arm_right_link_1_s*/ sphere_sphere_self_collision<decltype(q[0])>(ADD_3593, ADD_3594, ADD_3595, 0.095372, SUB_3916, ADD_3917, ADD_3918, 0.089005)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3632, ADD_3633, ADD_3634, 0.035041, ADD_3958, SUB_3959, ADD_3960, 0.153255)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3671, ADD_3672, ADD_3673, 0.035213, ADD_3958, SUB_3959, ADD_3960, 0.153255)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(SUB_3722, SUB_3723, SUB_3724, 0.034568, ADD_3958, SUB_3959, ADD_3960, 0.153255)){ return false; } } // (1645, 1645)
  if(/*arm_right_link_1_s vs. arm_right_link_3_e*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_3916, ADD_3917, ADD_3918, 0.089005, SUB_4226, SUB_4227, SUB_4228, 0.080043)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3958, SUB_3959, ADD_3960, 0.153255, ADD_4253, ADD_4254, ADD_4255, 0.095372)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3958, SUB_3959, ADD_3960, 0.153255, ADD_4286, ADD_4287, ADD_4288, 0.085433)){ return false; } } // (1645, 1645)
  if(/*arm_right_link_1_s vs. arm_right_link_5_r*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_3916, ADD_3917, ADD_3918, 0.089005, ADD_4600, ADD_4601, ADD_4602, 0.077046)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3958, SUB_3959, ADD_3960, 0.153255, ADD_4633, ADD_4634, ADD_4635, 0.079093)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3958, SUB_3959, ADD_3960, 0.153255, ADD_4675, ADD_4676, ADD_4677, 0.077046)){ return false; } } // (1645, 1645)
  if(/*arm_right_link_1_s vs. arm_right_link_6_b*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_3916, ADD_3917, ADD_3918, 0.089005, ADD_4762, ADD_4763, ADD_4764, 0.077881)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3958, SUB_3959, ADD_3960, 0.153255, ADD_4795, ADD_4796, ADD_4797, 0.077881)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3958, SUB_3959, ADD_3960, 0.153255, ADD_4828, ADD_4829, ADD_4830, 0.068019)){ return false; } } // (1645, 1645)
  if(/*arm_right_link_1_s vs. arm_right_link_7_t*/ sphere_sphere_self_collision<decltype(q[0])>(SUB_3916, ADD_3917, ADD_3918, 0.089005, ADD_4915, ADD_4916, ADD_4917, 0.068019)){ if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3958, SUB_3959, ADD_3960, 0.153255, ADD_4954, ADD_4955, ADD_4956, 0.035041)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3958, SUB_3959, ADD_3960, 0.153255, ADD_4993, ADD_4994, ADD_4995, 0.035213)){ return false; }
  if(sphere_sphere_self_collision<decltype(q[0])>(ADD_3958, SUB_3959, ADD_3960, 0.153255, SUB_5044, SUB_5045, SUB_5046, 0.034568)){ return false; } } // (1645, 1645)
  set_attachment_pose_hack(environment, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  if(attachment_environment_collision(environment)){ return false; } // (1645, 1646)
  }
  inline auto eefk(const std::array<float, 16> &q) noexcept -> std::array<float, 16> {
    return;
  }
}
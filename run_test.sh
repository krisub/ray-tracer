# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_cornell.json out1.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_dof_simple.json out2.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_glossy_reflection.json out3.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_motion_blur.json out4.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_soft_shadows.json out5.png &
# ./build/bin/ray -r 8 --path --caustic-photons 1000000 --caustic-radius 0.5 --caustic-gather 200 /u/krisub/graphics/p1/assets/scenes/caustics/glass_ball.json out1.png &
# ./build/bin/ray -r 8 --path --caustic-photons 1000000 --caustic-radius 0.5 --caustic-gather 200 /u/krisub/graphics/p1/assets/scenes/caustics/glass_cup.json out2.png &
# ./build/bin/ray -r 8 --path --caustic-photons 1000000 --caustic-radius 0.5 --caustic-gather 200 /u/krisub/graphics/p1/assets/scenes/caustics/metal_ring.json out4.png &

./build/bin/ray -r 8 /u/krisub/graphics/p1/assets/scenes/adv_shading/fresnel_test.json out_fresnel_test_no_fresnel.png
./build/bin/ray -r 8 /u/krisub/graphics/p1/assets/scenes/adv_shading/anisotropic_test.json out_anisotropic_test_no_fresnel.png
./build/bin/ray -r 8 /u/krisub/graphics/p1/assets/scenes/adv_shading/anisotropic_cylinders.json out_anisotropic_cylinders_no_fresnel.png
./build/bin/ray -r 8 --fresnel /u/krisub/graphics/p1/assets/scenes/adv_shading/fresnel_test.json out_fresnel_test_with_fresnel.png
./build/bin/ray -r 8 --fresnel /u/krisub/graphics/p1/assets/scenes/adv_shading/anisotropic_test.json out_anisotropic_test_with_fresnel.png
./build/bin/ray -r 8 --fresnel /u/krisub/graphics/p1/assets/scenes/adv_shading/anisotropic_cylinders.json out_anisotropic_cylinders_with_fresnel.png
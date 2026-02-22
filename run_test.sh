# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_cornell.json out1.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_dof_simple.json out2.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_glossy_reflection.json out3.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_motion_blur.json out4.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_soft_shadows.json out5.png &
# ./build/bin/ray -r 8 --path --caustic-photons 1000000 --caustic-radius 0.5 --caustic-gather 200 /u/krisub/graphics/p1/assets/scenes/caustics/glass_ball.json out1.png &
# ./build/bin/ray -r 8 --path --caustic-photons 1000000 --caustic-radius 0.5 --caustic-gather 200 /u/krisub/graphics/p1/assets/scenes/caustics/glass_cup.json out2.png &
./build/bin/ray -r 8 --path --caustic-photons 1000000 --caustic-radius 0.5 --caustic-gather 200 /u/krisub/graphics/p1/assets/scenes/caustics/metal_ring.json out4.png &
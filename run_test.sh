# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_cornell.json out1.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_dof_simple.json out2.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_glossy_reflection.json out3.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_motion_blur.json out4.png &
# ./build/bin/ray -r 10 --path -j /u/krisub/graphics/p1/ray/settings.json /u/krisub/graphics/p1/assets/scenes/global_ill_path/gi_soft_shadows.json out5.png &
# ./build/bin/ray -r 8 --path --caustic-photons 1000000 --caustic-radius 0.5 --caustic-gather 200 /u/krisub/graphics/p1/assets/scenes/caustics/glass_ball.json out1.png &
# ./build/bin/ray -r 8 --path --caustic-photons 1000000 --caustic-radius 0.5 --caustic-gather 200 /u/krisub/graphics/p1/assets/scenes/caustics/glass_cup.json out2.png &
# ./build/bin/ray -r 8 --path --caustic-photons 1000000 --caustic-radius 0.5 --caustic-gather 200 /u/krisub/graphics/p1/assets/scenes/caustics/metal_ring.json out4.png &

# ./build/bin/ray -r 8 -c /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.png /u/krisub/graphics/p1/assets/scenes/adv_shading/fresnel_test.json out_fresnel_test_no_fresnel.png
# ./build/bin/ray -r 8 -c /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.png /u/krisub/graphics/p1/assets/scenes/adv_shading/anisotropic_test.json out_anisotropic_test_no_fresnel.png
# ./build/bin/ray -r 8 -c /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.png /u/krisub/graphics/p1/assets/scenes/adv_shading/anisotropic_cyl.json out_anisotropic_cyl_no_fresnel.png
# ./build/bin/ray -r 8 --fresnel -c /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.png /u/krisub/graphics/p1/assets/scenes/adv_shading/fresnel_test.json out_fresnel_test_with_fresnel.png
# ./build/bin/ray -r 8 --fresnel -c /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.png /u/krisub/graphics/p1/assets/scenes/adv_shading/anisotropic_test.json out_anisotropic_test_with_fresnel.png
# ./build/bin/ray -r 8 --fresnel -c /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.png /u/krisub/graphics/p1/assets/scenes/adv_shading/anisotropic_cyl.json out_anisotropic_cyl_with_fresnel.png

# ./build/bin/ray -r 8 /u/krisub/graphics/p1/assets/scenes/adv_shading/fresnel_rim.json out_fresnel_rim_no_fresnel.png
# ./build/bin/ray -r 8 --fresnel /u/krisub/graphics/p1/assets/scenes/adv_shading/fresnel_rim.json out_fresnel_rim_with_fresnel.png

# ./build/bin/ray -r 8 -c /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.png ./ecm2/extra_credit_scenes/adv_shading/fresnel_rim.json ./ecm2/extra_credit_output/adv_shading/no_fresnel_rim.png
# ./build/bin/ray -r 8 ./ecm2/extra_credit_scenes/adv_shading/anisotropic_test.json ./ecm2/extra_credit_output/adv_shading/no_anisotropic_test.png
# ./build/bin/ray -r 8 ./ecm2/extra_credit_scenes/adv_shading/anisotropic_cyl.json ./ecm2/extra_credit_output/adv_shading/no_anisotropic_cyl.png
# ./build/bin/ray -r 8 --fresnel -c /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.png ./ecm2/extra_credit_scenes/adv_shading/fresnel_rim.json ./ecm2/extra_credit_output/adv_shading/with_fresnel_rim.png
# ./build/bin/ray -r 8 --fresnel ./ecm2/extra_credit_scenes/adv_shading/anisotropic_test.json ./ecm2/extra_credit_output/adv_shading/with_anisotropic_test.png
# ./build/bin/ray -r 8 --fresnel ./ecm2/extra_credit_scenes/adv_shading/anisotropic_cyl.json ./ecm2/extra_credit_output/adv_shading/with_anisotropic_cyl.png


# ./build/bin/ray -r 5 -c /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.png ./ecm2/extra_credit_scenes/volumetric/fog.json ./ecm2/extra_credit_output/volumetric/fog.png
# ./build/bin/ray -r 5 -c /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.png ./ecm2/extra_credit_scenes/volumetric/smoke.json ./ecm2/extra_credit_output/volumetric/smoke.png
# ./build/bin/ray -r 5 -c /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.png ./ecm2/extra_credit_scenes/volumetric/nebula.json ./ecm2/extra_credit_output/volumetric/nebula.png
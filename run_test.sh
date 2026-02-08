cd build
make -j8
cd ..
./build/bin/ray -r 5 -t 8 /Users/krisub/Projects/graphics/ray_tracer/ray/my_out/overlap_shadows.json ../output_imgs/overlap_shadows.png
./build/bin/ray -r 5 -t 8 --tir /Users/krisub/Projects/graphics/ray_tracer/ray/my_out/overlap_shadows.json ../output_imgs/overlap_shadows_tir.png
./build/bin/ray -r 5 -t 8 --handle-overlap /Users/krisub/Projects/graphics/ray_tracer/ray/my_out/overlap_shadows.json ../output_imgs/overlap_shadows_handle_overlap.png
./build/bin/ray -r 5 -t 8 --tir --handle-overlap /Users/krisub/Projects/graphics/ray_tracer/ray/my_out/overlap_shadows.json ../output_imgs/overlap_shadows_tir_handle_overlap.png
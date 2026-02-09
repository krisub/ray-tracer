cd build
make -j8
cd ..
./build/bin/ray -r 5 -t 8 /Users/krisub/Projects/graphics/ray_tracer/scenes/hitchcock.json ../output_imgs/hitchcock_noaa.png
# ./build/bin/ray -r 5 -t 8 --aaa /Users/krisub/Projects/graphics/ray_tracer/scenes/hitchcock.json ../output_imgs/hitchcock_aaa.png
# ./build/bin/ray -r 5 -t 8 --fxaa /Users/krisub/Projects/graphics/ray_tracer/scenes/hitchcock.json ../output_imgs/hitchcock_fxaa.png

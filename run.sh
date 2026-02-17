python3 raycheck.py --refbin ./ray-solution --json settings.json --timelimit 600
mv /u/krisub/graphics/p1/ray/raycheck.out/refcache/* /u/krisub/graphics/p1/ray/noaa
rm -rf raycheck.out

python3 raycheck.py --refbin ./ray-solution --cubemap /u/krisub/graphics/p1/cubemaps/cubemap_autumn/autumn_negative_x.png --json settings.json --timelimit 600
for f in /u/krisub/graphics/p1/ray/raycheck.out/refcache/*; do
    mv "$f" "/u/krisub/graphics/p1/ray/noaa/autumn_$(basename "$f")"
done
rm -rf raycheck.out

python3 raycheck.py --refbin ./ray-solution --cubemap /u/krisub/graphics/p1/cubemaps/cubemap_brudslojan/negx.jpg --json settings.json --timelimit 600
for f in /u/krisub/graphics/p1/ray/raycheck.out/refcache/*; do
    mv "$f" "/u/krisub/graphics/p1/ray/noaa/brudslojan_$(basename "$f")"
done
rm -rf raycheck.out

python3 raycheck.py --refbin ./ray-solution --cubemap /u/krisub/graphics/p1/cubemaps/cubemap_iceriver/negx.jpg --json settings.json --timelimit 600
for f in /u/krisub/graphics/p1/ray/raycheck.out/refcache/*; do
    mv "$f" "/u/krisub/graphics/p1/ray/noaa/iceriver_$(basename "$f")"
done
rm -rf raycheck.out

python3 raycheck.py --refbin ./ray-solution --cubemap /u/krisub/graphics/p1/cubemaps/cubemap_nvlobby_new/nvlobby_new_negative_x.png --json settings.json --timelimit 600
for f in /u/krisub/graphics/p1/ray/raycheck.out/refcache/*; do
    mv "$f" "/u/krisub/graphics/p1/ray/noaa/nvlobby_$(basename "$f")"
done
rm -rf raycheck.out

python3 raycheck.py --refbin ./ray-solution --cubemap /u/krisub/graphics/p1/cubemaps/cubemap_terrain/terrain_negative_x.png --json settings.json --timelimit 600
for f in /u/krisub/graphics/p1/ray/raycheck.out/refcache/*; do
    mv "$f" "/u/krisub/graphics/p1/ray/noaa/terrain_$(basename "$f")"
done
rm -rf raycheck.out

python3 raycheck.py --refbin ./ray-solution --cubemap /u/krisub/graphics/p1/cubemaps/cubemap_unwrapped/unwrapped_negative_x.png --json settings.json --timelimit 600
for f in /u/krisub/graphics/p1/ray/raycheck.out/refcache/*; do
    mv "$f" "/u/krisub/graphics/p1/ray/noaa/unwrapped_$(basename "$f")"
done
rm -rf raycheck.out

python3 raycheck.py --refbin ./ray-solution --cubemap /u/krisub/graphics/p1/cubemaps/earth/negx.png --json settings.json --timelimit 600
for f in /u/krisub/graphics/p1/ray/raycheck.out/refcache/*; do
    mv "$f" "/u/krisub/graphics/p1/ray/noaa/earth_$(basename "$f")"
done
rm -rf raycheck.out

python3 raycheck.py --refbin ./ray-solution --cubemap /u/krisub/graphics/p1/cubemaps/escher/negx.png --json settings.json --timelimit 600
for f in /u/krisub/graphics/p1/ray/raycheck.out/refcache/*; do
    mv "$f" "/u/krisub/graphics/p1/ray/noaa/escher_$(basename "$f")"
done
rm -rf raycheck.out

python3 raycheck.py --refbin ./ray-solution --cubemap /u/krisub/graphics/p1/cubemaps/ForbiddenCity/negx.png --json settings.json --timelimit 600
for f in /u/krisub/graphics/p1/ray/raycheck.out/refcache/*; do
    mv "$f" "/u/krisub/graphics/p1/ray/noaa/forbiddencity_$(basename "$f")"
done
rm -rf raycheck.out
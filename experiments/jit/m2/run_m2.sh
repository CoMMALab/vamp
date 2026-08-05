#!/usr/bin/env bash
# M2 driver: sweep scenes, time the scene-specialization compile (clang bar), and
# run the isolated-kernel + robot FK/CC benches. Run inside the jit_patch env:
#   micromamba run -n jit_patch bash experiments/jit/m2/run_m2.sh
set -euo pipefail

VAMP=/home/zak/src/vamp_jit/vamp
M2=$VAMP/experiments/jit/m2
RES=$VAMP/experiments/jit/results
PREFIX=/home/zak/micromamba/envs/jit_patch
BUILD=/tmp/claude-1000/-home-zak-src-vamp-jit/3dcbbe7c-40b1-4721-b12b-3292533b6ada/scratchpad/m2build
mkdir -p "$BUILD"
PDQ=$(find "$VAMP/build" -name pdqsort.h -printf '%h\n' 2>/dev/null | head -1)

CXX="clang++ -std=c++17 -O3 -march=native -fno-strict-aliasing"
INC="-I $VAMP/src/impl -isystem $PREFIX/include/eigen3 -I $PDQ -I $BUILD -I $M2"
SEED=12345

# Build the generator once.
$CXX "$M2/m2_gen.cc" -o "$BUILD/m2_gen"

LAT=$RES/m2_spec_latency.csv
KER=$RES/m2_kernel.csv
ROB=$RES/m2_robot.csv
echo "N,extent,kernel_only_compile_min_s,repeats" > "$LAT"
echo "kernel,N,extent,rake,k_queries,dist,med_ns_generic,med_ns_spec,speedup,collision_frac,mismatches" > "$KER"
echo "robot,n_spheres,dim,N,extent,rake,m_blocks,t0_fused_ns,t0_split_ns,t2_split_ns,speedup_split,speedup_vs_fused,collision_frac,sphere_mismatch,verdict_mismatch" > "$ROB"

compile_min() {  # $1=src $2=out ; echoes min wall of 3 compiles (awk float math, no bc)
    local t0 t1 dts=""
    for _ in 1 2 3; do
        t0=$(date +%s.%N)
        $CXX $INC -c "$1" -o "$2" 2>/dev/null
        t1=$(date +%s.%N)
        dts+=" $(awk "BEGIN{printf \"%.4f\", $t1-$t0}")"
    done
    awk "BEGIN{m=1e9; n=split(\"$dts\",a,\" \"); for(i=1;i<=n;i++) if(a[i]!=\"\"&&a[i]<m)m=a[i]; printf \"%.4f\", m}"
}

ROBOT_NS=" 50 200 800 "
for extent in 0.6 1.5; do
    for N in 10 25 50 100 200 400 800 1600; do
        "$BUILD/m2_gen" "$N" "$extent" "$SEED" "$BUILD/scene_gen.hh"

        # (1) scene-specialization compile latency (kernel-only TU).
        lat=$(compile_min "$M2/m2_kernel_only.cc" "$BUILD/kern.o")
        echo "$N,$extent,$lat,3" >> "$LAT"
        echo "[m2] N=$N extent=$extent spec_compile_min=${lat}s"

        # (2) isolated-kernel throughput + correctness.
        $CXX $INC "$M2/m2_kernel_bench.cc" -o "$BUILD/m2_kernel_bench" 2>/dev/null
        "$BUILD/m2_kernel_bench" >> "$KER"

        # (3) robot FK/CC bench at a few representative N.
        if [[ "$ROBOT_NS" == *" $N "* ]]; then
            $CXX $INC "$M2/m2_robot_bench.cc" -o "$BUILD/m2_robot_bench" 2>/dev/null
            "$BUILD/m2_robot_bench" >> "$ROB"
        fi
    done
done

echo "[m2] done. wrote:"; echo "  $LAT"; echo "  $KER"; echo "  $ROB"

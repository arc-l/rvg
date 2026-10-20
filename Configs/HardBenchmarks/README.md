# Organic random-polygon hard benchmark v3

This is the default 20-map paper workload. All maps use the original rectangular robot and fixed query, are generated only from seeded random polygons, and pass independent static-RVG checks at every benchmark orientation resolution. Sixteen protected general maps are ranked without lattice representability data. Four disclosed unprotected resolution-stress maps cover two ground-truth invalid-start and two invalid-goal cases at 0.1 m and 8 orientation bins.

These fixtures and their historical selection metadata were copied from
`arc-l/drvg` at `53daeef0`. The generation/audit workflow remains in that
repository. To measure the static planner here, build
`mainBenchmarkVertexPropagation` and run from the RVG repository root:

```bash
./build/cpu/mainBenchmarkVertexPropagation Configs/HardBenchmarks build/cpu/propagation.csv 20 5 24 90,180,360 cpu_reference,cpu
```

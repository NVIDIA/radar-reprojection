# sample_data

One Physical AI AV clip used as the integration-test fixture for the
Python `PhysicalAIAVAdapter` (`examples/python/physical_ai_av_adapter.py`)
and as the default clip for the `run_clip.py` end-to-end demo.

**Clip id:** `25cd4769-5dcf-4b53-a351-bf2c5deb6124`
**Chunk:** `0`
**Dataset revision:** `dfcc35f941c38f050e9ce256a4c0aff9e33615b9`

## Layout (mirrors the Hugging Face repo)

```
sample_data/
├── radar/radar_front_center_srr_0/
│   └── 25cd4769-...radar_front_center_srr_0.parquet    # one radar, one clip
├── labels/egomotion/
│   └── 25cd4769-...egomotion.parquet                    # 111 Hz ego pose + velocity + accel
├── labels/obstacle.offline/
│   └── 25cd4769-...obstacle.offline.parquet             # per-timestamp actor poses (in rig frame)
├── calibration/sensor_extrinsics/
│   └── sensor_extrinsics.chunk_0000.parquet             # whole chunk, ~100 clips worth (63 KB)
└── calibration/vehicle_dimensions/
    └── vehicle_dimensions.chunk_0000.parquet            # whole chunk (9 KB)
```

Total on disk: ~1.6 MB (vs. ~180 MB for the full chunks in the HF cache).

## What's missing vs. production

- **No non-offline `obstacle` feature.** The current dataset only publishes `obstacle.offline`.
- **Calibration parquets are per-chunk, not per-clip.** They index by `clip_id`; the adapter filters at load time.

Schemas are documented in [`../docs/SCHEMA.md`](../docs/SCHEMA.md).

## License

Dataset is gated under the NVIDIA Physical AI AV license (<https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles>). Do not redistribute. The repo's `.gitignore` excludes `sample_data/` from commits by default.

## Reproducing

```bash
conda activate radar_reprojection
python ../scripts/download_sample_clip.py                 # defaults
python ../scripts/download_sample_clip.py --radars all    # every radar in the clip
python ../scripts/download_sample_clip.py --clip auto     # pick a new clip
python ../scripts/download_sample_clip.py --help
```

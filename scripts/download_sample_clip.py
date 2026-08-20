#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Download one Physical AI AV clip into a local sample_data/ directory.

Mirrors the Hugging Face repo layout so the Python adapter can point at
either `sample_data/` or a full dataset checkout without code changes.

Prereqs:
    conda activate radar_reprojection
    hf auth login        # accept the dataset license first

Examples:
    # default: front-center SRR radar for the known sample clip
    python scripts/download_sample_clip.py

    # different clip, multiple radars, custom output
    python scripts/download_sample_clip.py \\
        --clip b1234567-... \\
        --radars radar_front_center_srr_0 radar_side_left_srr_0 \\
        --out /tmp/my_sample

    # all radars present in the clip
    python scripts/download_sample_clip.py --radars all

    # pick the first clip that has all requested features
    python scripts/download_sample_clip.py --clip auto

    # download the 5 clips with the HIGHEST perception quality_score (NuRec ratings)
    python scripts/download_sample_clip.py --top-k 5 --radars all

    # ...or the 5 LOWEST-quality clips
    python scripts/download_sample_clip.py --top-k 5 --order lowest --radars all

    # estimate download size first
    python scripts/download_sample_clip.py --radars all --dry-run
"""
from __future__ import annotations

import argparse
import pathlib
import shutil
import sys
import zipfile

DEFAULT_CLIP = "25cd4769-5dcf-4b53-a351-bf2c5deb6124"
DEFAULT_RADARS = ["radar_front_center_srr_0"]
REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_OUT = REPO_ROOT / "sample_data"

CALIB_FEATURES = ["sensor_extrinsics", "vehicle_dimensions"]
LABEL_FEATURES = ["egomotion", "obstacle.offline"]

# The base dataset carries no quality/perception score. The sibling "NuRec"
# (Neural Reconstruction) dataset ships a per-clip perception `quality_score`
# in a repo-root `clip_ratings_<version>.csv`, keyed by the same clip UUIDs.
# See --top-k / --order below.
NUREC_REPO_ID = "nvidia/PhysicalAI-Autonomous-Vehicles-NuRec"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--clip", default=DEFAULT_CLIP,
                   help=f"Clip UUID, or 'auto' to pick the first clip with all requested features. "
                        f"Default: {DEFAULT_CLIP}")
    p.add_argument("--radars", nargs="+", default=DEFAULT_RADARS,
                   help="Radar feature names (e.g. radar_front_center_srr_0), or 'all' for every "
                        "radar present in the clip. Default: %(default)s")
    p.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT,
                   help=f"Output directory. Default: {DEFAULT_OUT.relative_to(REPO_ROOT)}/")
    p.add_argument("--top-k", type=int, default=None, metavar="K",
                   help="Instead of a single --clip, download the K clips with the highest "
                        "perception quality_score (see --order for lowest). Scores come from the "
                        f"{NUREC_REPO_ID} 'clip_ratings' CSV; candidates are restricted to clips "
                        "present in this dataset with the requested radars + labels + calibration. "
                        "Overrides --clip. NOTE: NuRec-rated clips use the srr_3/mrr_2 radar "
                        "config, so pair with '--radars all' — the default radar_front_center_srr_0 "
                        "matches no rated clip.")
    p.add_argument("--order", choices=["highest", "lowest"], default="highest",
                   help="With --top-k, take the highest- or lowest-quality clips. Default: %(default)s")
    p.add_argument("--dry-run", action="store_true",
                   help="Print download sizes and exit, don't fetch.")
    return p.parse_args()


def pick_auto_clip(ds, required_features: list[str]) -> str:
    mask = ds.feature_presence[required_features].all(axis=1)
    if not mask.any():
        sys.exit(f"no clip has all of: {required_features}")
    return ds.feature_presence.index[mask][0]


def load_quality_ratings():
    """Load the NuRec per-clip perception `quality_score` table, indexed by clip_id.

    The base dataset has no score; ratings live in a repo-root `clip_ratings_<version>.csv`
    in `NUREC_REPO_ID`. The filename is version-stamped, so discover the latest at runtime.
    Returns `(ratings_filename, DataFrame[quality_score, time_of_day, ego_speed])`.
    """
    import pandas as pd
    from physical_ai_av import HfRepoInterface

    nurec = HfRepoInterface(repo_id=NUREC_REPO_ID, repo_type="dataset")
    csvs = sorted(
        item.path
        for item in nurec.api.list_repo_tree(path_in_repo="", recursive=False,
                                             **nurec.repo_snapshot_info)
        if item.path.startswith("clip_ratings_") and item.path.endswith(".csv")
    )
    if not csvs:
        sys.exit(f"no clip_ratings_*.csv found in {NUREC_REPO_ID}")
    ratings_file = csvs[-1]  # highest version string
    df = pd.read_csv(nurec.download_file(ratings_file)).set_index("clip_id")
    return ratings_file, df


def pick_top_quality_clips(ds, ratings, requested_radars: list[str], k: int, order: str):
    """Return the K (clip_id, quality_score) pairs with the highest/lowest `quality_score`.

    Candidates are restricted to rated clips that are present in this dataset AND carry all
    required features (labels + calibration + the requested radars), so every returned clip is
    actually downloadable. `order` is "highest" or "lowest".
    """
    fp = ds.feature_presence
    unknown = [r for r in requested_radars if r != "all" and r not in fp.columns]
    if unknown:
        sys.exit(f"unknown radar feature(s): {unknown}")

    candidates = ratings.index.intersection(fp.index)
    mask = fp.loc[candidates, LABEL_FEATURES + CALIB_FEATURES].all(axis=1)
    if requested_radars == ["all"]:
        radar_cols = [c for c in fp.columns if c.startswith("radar_")]
        mask &= fp.loc[candidates, radar_cols].any(axis=1)
    else:
        mask &= fp.loc[candidates, requested_radars].all(axis=1)

    eligible = mask.index[mask.to_numpy()]
    if len(eligible) == 0:
        sys.exit(f"no rated clip has all required features {LABEL_FEATURES + CALIB_FEATURES} + "
                 f"radars {requested_radars}. NuRec-rated clips do not include "
                 f"'radar_front_center_srr_0'; try '--radars all'.")
    if len(eligible) < k:
        print(f"warning: only {len(eligible)} rated clip(s) match the requested features "
              f"(fewer than --top-k {k})")

    ranked = ratings.loc[eligible, "quality_score"].sort_values(ascending=(order == "lowest"))
    return list(ranked.head(k).items())


def resolve_radars(ds, clip_id: str, requested: list[str]) -> list[str]:
    available = [c for c in ds.feature_presence.columns if c.startswith("radar_")
                 and ds.feature_presence.at[clip_id, c]]
    if requested == ["all"]:
        return available
    missing = [r for r in requested if r not in available]
    if missing:
        sys.exit(f"clip {clip_id} is missing radars: {missing}\navailable: {available}")
    return requested


def chunk_filenames(ds, clip_id: str, features: list[str]) -> list[str]:
    chunk = ds.get_clip_chunk(clip_id)
    return [ds.features.get_chunk_feature_filename(chunk, f) for f in features]


def extract_clip_parquets_from_zip(zip_path: pathlib.Path, clip_id: str, dst_dir: pathlib.Path) -> list[pathlib.Path]:
    dst_dir.mkdir(parents=True, exist_ok=True)
    written: list[pathlib.Path] = []
    with zipfile.ZipFile(zip_path, "r") as zf:
        for name in zf.namelist():
            if clip_id not in name:
                continue
            dst = dst_dir / pathlib.Path(name).name
            with zf.open(name) as src, open(dst, "wb") as f:
                shutil.copyfileobj(src, f)
            written.append(dst)
    return written


def download_clip(ds, clip_id: str, radars: list[str], out: pathlib.Path, dry_run: bool) -> None:
    """Plan (and, unless `dry_run`, fetch + extract) one clip's radars + labels + calibration."""
    print(f"\nclip:   {clip_id}")
    print(f"chunk:  {ds.get_clip_chunk(clip_id)}")
    print(f"radars: {radars}")

    # Plan downloads
    all_features = radars + LABEL_FEATURES + CALIB_FEATURES
    files = chunk_filenames(ds, clip_id, all_features)

    info = ds.download_files(files, dry_run=True)
    print("\ndownload plan:")
    for i in info:
        marker = "cached" if i.is_cached else "fetch "
        print(f"  [{marker}] {i.file_size/1e6:>8.2f} MB  {i.filename}")
    total_mb = sum(i.file_size for i in info if i.will_download) / 1e6
    print(f"  {'':8}  {'':>8}      new bytes to fetch: {total_mb:.2f} MB")

    if dry_run:
        return

    # Fetch chunks
    ds.download_files(files)

    # Extract per-clip parquets from the zip features. Mirror the HF repo
    # layout so sample_data/radar/<radar>/... matches what the loader expects.
    for feat in radars + LABEL_FEATURES:
        chunk_file = ds.features.get_chunk_feature_filename(ds.get_clip_chunk(clip_id), feat)
        zip_path = pathlib.Path(ds.download_file(chunk_file))
        dst = out / pathlib.Path(chunk_file).parent
        written = extract_clip_parquets_from_zip(zip_path, clip_id, dst)
        for w in written:
            print(f"extracted  {w.relative_to(out)}  ({w.stat().st_size/1e6:.2f} MB)")

    # Copy calibration chunk parquets as-is (small, clip_id-indexed inside)
    for feat in CALIB_FEATURES:
        chunk_file = ds.features.get_chunk_feature_filename(ds.get_clip_chunk(clip_id), feat)
        src = pathlib.Path(ds.download_file(chunk_file))
        dst_dir = out / pathlib.Path(chunk_file).parent
        dst_dir.mkdir(parents=True, exist_ok=True)
        dst = dst_dir / src.name
        shutil.copyfile(src, dst)
        print(f"copied     {dst.relative_to(out)}  ({dst.stat().st_size/1e3:.2f} KB)")


def main() -> None:
    args = parse_args()

    try:
        from physical_ai_av import PhysicalAIAVDatasetInterface
    except ImportError:
        sys.exit("physical_ai_av not installed. Run: conda activate radar_reprojection")

    ds = PhysicalAIAVDatasetInterface()

    if args.top_k is not None:
        if args.top_k <= 0:
            sys.exit("--top-k must be a positive integer")
        ratings_file, ratings = load_quality_ratings()
        selected = pick_top_quality_clips(ds, ratings, args.radars, args.top_k, args.order)
        print(f"quality ratings: {NUREC_REPO_ID}/{ratings_file}")
        print(f"selected {len(selected)} clip(s) by {args.order} quality_score:")
        for cid, score in selected:
            print(f"  quality_score={score:6.2f}  {cid}")
        for cid, _ in selected:
            download_clip(ds, cid, resolve_radars(ds, cid, args.radars), args.out, args.dry_run)
        if not args.dry_run:
            print(f"\ndone -> {args.out}")
        return

    if args.clip == "auto":
        initial = args.radars if args.radars != ["all"] else []
        required = LABEL_FEATURES + CALIB_FEATURES + initial
        clip_id = pick_auto_clip(ds, required)
        print(f"auto-picked clip: {clip_id}")
    else:
        clip_id = args.clip

    radars = resolve_radars(ds, clip_id, args.radars)
    download_clip(ds, clip_id, radars, args.out, args.dry_run)
    if not args.dry_run:
        print(f"\ndone -> {args.out}")


if __name__ == "__main__":
    main()

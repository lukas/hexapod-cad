#!/usr/bin/env python3
"""Mirror a BuildViz build's current default version to the cloud hub.

Analysis pages whose ``sourceVersion`` matches that default version are
mirrored after the build, so a locally verified FEA page does not disappear
when the build is viewed on the cloud hub.

Standing convention (Lukas, Aug 2026): every local `verify-buildviz` publish
should also land on the CoreWeave-hosted BuildViz hub so the build is
viewable off this machine:

    https://buildviz.cwd1f0-new-cluster.coreweave.app/?project=prototype_sts3215&build=prototype_sts3215

This script reads the LOCAL hub cache (~/.buildviz/cache/<build-id>), takes
whatever version is currently the default (e.g. v64), and POSTs it to the
remote hub's /__buildviz/push with all mesh bytes attached (self-contained,
same payload the deploy/coreweave/migrate_local_builds.py migration uses)
plus the design spec, then marks it the remote default.

Auth: the remote hub requires X-API-Key.  The key lives in BUILDVIZ_API_KEY
(same key as the local hub; canonical source = the CoreWeave k8s secret
`buildviz-api-key`, readable via
    KUBECONFIG=~/.kube/coreweave.yaml kubectl get secret buildviz-api-key \
        -o jsonpath='{.data.key}' | base64 -d
).

Called from the Makefile at the end of `verify-buildviz` (failure-tolerant:
a dead network must not fail the local verify), or standalone:

    BUILDVIZ_API_KEY=... uv run python tools/push_cloud_buildviz.py \
        [--build-id <id>]
"""

import argparse
import base64
import json
import os
import sys
import urllib.request
from pathlib import Path

DEFAULT_CLOUD_URL = "https://buildviz.cwd1f0-new-cluster.coreweave.app"
DEFAULT_BUILD_ID = "prototype_sts3215"
MAX_UPLOAD_BYTES = 256 * 1024 * 1024

BUILDVIZ_HOME = Path.home() / ".buildviz"
CACHE = BUILDVIZ_HOME / "cache"
ASSETS = CACHE / "_assets"


def _resolve_mesh_file(url: str, scene_dir: Path) -> Path | None:
    if url.startswith(("http://", "https://")):
        return None
    if url.startswith("/builds/_assets/"):
        candidate = ASSETS / Path(url).name
        return candidate if candidate.exists() else None
    if url.startswith("/"):
        return None
    candidate = scene_dir / url.lstrip("./")
    return candidate if candidate.exists() else None


def _assets_for_scene(scene: dict, scene_dir: Path) -> list[dict]:
    assets = []
    for mesh in scene.get("meshes", []):
        url, mesh_id = mesh.get("url", ""), mesh.get("id", "")
        if not url or not mesh_id:
            continue
        file = _resolve_mesh_file(url, scene_dir)
        if file is not None:
            assets.append({
                "meshId": mesh_id,
                "data": base64.b64encode(file.read_bytes()).decode(),
                "ext": file.suffix.lstrip(".") or "stl",
            })
    return assets


def _post_json(url: str, api_key: str, payload: dict) -> dict:
    req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "X-API-Key": api_key},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=300) as response:
        return json.load(response)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--build-id", default=DEFAULT_BUILD_ID)
    ap.add_argument("--cloud-url",
                    default=os.environ.get("BUILDVIZ_CLOUD_URL",
                                           DEFAULT_CLOUD_URL))
    args = ap.parse_args()
    base = args.cloud_url.rstrip("/")

    api_key = os.environ.get("BUILDVIZ_API_KEY", "")
    if not api_key:
        print("push_cloud_buildviz: BUILDVIZ_API_KEY not set -- skipping "
              "cloud mirror (see docstring for the key source)",
              file=sys.stderr)
        return 1

    status = json.load(urllib.request.urlopen(
        f"{base}/__buildviz/status", timeout=30))
    if status.get("service") != "buildviz-hub":
        print(f"push_cloud_buildviz: {base} is not a BuildViz hub",
              file=sys.stderr)
        return 1

    build_dir = CACHE / args.build_id
    meta = json.loads((build_dir / "meta.json").read_text())
    version = meta.get("defaultVersion", "main")
    scene_file = build_dir / "scene.json"
    scene = json.loads(scene_file.read_text())

    # Staleness guard (incident 2026-08-26: `buildviz register` serves the
    # live buildDir but does NOT refresh this push-cache, so the mirror
    # re-shipped a day-old snapshot).  If the registered buildDir has a
    # newer scene.json than the cache, refuse to mirror the stale copy.
    registry = BUILDVIZ_HOME / "registry.json"
    if registry.exists():
        for entry in json.loads(registry.read_text()).get("builds", []):
            if entry.get("id") != args.build_id:
                continue
            live = Path(entry.get("buildDir", "")) / "scene.json"
            if live.exists() and \
                    live.stat().st_mtime > scene_file.stat().st_mtime + 1.0:
                print(f"push_cloud_buildviz: cache for '{args.build_id}' is "
                      f"STALE -- the registered buildDir has a newer "
                      f"scene.json ({live}).\nRun `npx buildviz push "
                      f"--build-id {args.build_id} --upload-assets --scene "
                      f"{live}` first, then re-run this mirror.",
                      file=sys.stderr)
                return 1

    assets = _assets_for_scene(scene, build_dir)

    # The hub REQUIRES a version message naming the change. Mirror the local
    # version's own message when it has one; otherwise say what this is.
    message = next(
        (entry.get("message") for entry in meta.get("versions", [])
         if entry.get("name") == version and entry.get("message")),
        None,
    ) or f"mirror of local {args.build_id}@{version}"

    payload = {
        "buildId": args.build_id,
        "version": version,
        "scene": scene,
        "setDefault": True,
        "message": message,
        "maxUploadBytes": MAX_UPLOAD_BYTES,
        "assets": assets,
    }
    name = meta.get("name")
    if name:
        payload["name"] = name
    spec = build_dir / "design_spec.yaml"
    if spec.exists():
        payload["designSpec"] = spec.read_text()

    result = _post_json(f"{base}/__buildviz/push", api_key, payload)
    print(f"    cloud mirror: {result.get('summary', 'ok')} -> "
          f"{base}/?project={args.build_id.split('/')[0]}"
          f"&build={args.build_id.split('/')[-1]}")

    analyses_dir = build_dir / "analyses"
    if analyses_dir.exists():
        for analysis_dir in sorted(p for p in analyses_dir.iterdir()
                                   if p.is_dir()):
            analysis_scene_file = analysis_dir / "scene.json"
            analysis_meta_file = analysis_dir / "meta.json"
            if not analysis_scene_file.exists() or not analysis_meta_file.exists():
                continue
            analysis_meta = json.loads(analysis_meta_file.read_text())
            if analysis_meta.get("sourceVersion") != version:
                continue
            analysis_scene = json.loads(analysis_scene_file.read_text())
            analysis_payload = {
                "buildId": args.build_id,
                "name": analysis_meta.get("name") or analysis_dir.name,
                "displayName": analysis_meta.get("displayName"),
                "message": analysis_meta.get("message"),
                "sourceVersion": version,
                "sourceBranch": analysis_meta.get("sourceBranch"),
                "scene": analysis_scene,
                "assets": _assets_for_scene(analysis_scene, analysis_dir),
                "maxUploadBytes": MAX_UPLOAD_BYTES,
            }
            analysis_result = _post_json(
                f"{base}/__buildviz/push-analysis", api_key,
                analysis_payload,
            )
            print("    cloud analysis: "
                  f"{analysis_result.get('summary', analysis_dir.name)} -> "
                  f"{base}/?project={args.build_id.split('/')[0]}"
                  f"&build={args.build_id.split('/')[-1]}"
                  f"&analysis={analysis_dir.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

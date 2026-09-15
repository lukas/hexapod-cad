#!/usr/bin/env python3
"""Upload the connected one-leg STEP load path to Onshape for simulation.

This is intentionally a thin project helper around the Onshape REST API. It
keeps credentials in ``.secrets/onshape.env`` and writes a JSON artifact with
the created Part Studio, Assembly, and URL.
"""

from __future__ import annotations

import argparse
import base64
import json
import mimetypes
import os
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
CAD_TEST = ROOT / "hexapod_walker" / "prototype_sts3215" / "cad_step_test"
ARTIFACT_DIR = ROOT / "hexapod_walker" / "prototype_sts3215" / "artifacts" / "onshape"

DID = "a1754ab4e10f7c6d841687a0"
WID = "303c62c71765c7d05d5cc151"
PETG_TEMPLATE_EID = "a4274d2ccf3024e7b378aead"
MATERIAL_PROPERTY_ID = "57f3fb8efa3416c06701d615"


def load_env(path: Path) -> None:
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def basic_auth_header() -> str:
    key = os.environ["ONSHAPE_ACCESS_KEY"]
    secret = os.environ["ONSHAPE_SECRET_KEY"]
    return "Basic " + base64.b64encode(f"{key}:{secret}".encode()).decode()


class OnshapeClient:
    def __init__(self, base_url: str):
        self.base_url = base_url.rstrip("/")
        self.auth = basic_auth_header()

    def request(
        self,
        path: str,
        *,
        method: str = "GET",
        data: bytes | dict | list | None = None,
        headers: dict[str, str] | None = None,
        timeout: int = 60,
    ) -> dict:
        req_headers = {
            "Authorization": self.auth,
            "Accept": "application/json;charset=UTF-8; qs=0.09",
        }
        if headers:
            req_headers.update(headers)
        body = data
        if isinstance(data, (dict, list)):
            body = json.dumps(data).encode()
            req_headers.setdefault("Content-Type", "application/json;charset=UTF-8")
        request = urllib.request.Request(
            self.base_url + path,
            data=body,
            method=method,
            headers=req_headers,
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                payload = response.read()
                if not payload:
                    return {"_status": response.status}
                text = payload.decode("utf-8", errors="replace")
                try:
                    parsed = json.loads(text)
                except json.JSONDecodeError:
                    parsed = {"_text": text[:2000]}
                if isinstance(parsed, dict):
                    parsed.setdefault("_status", response.status)
                return parsed
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"{method} {path} failed {exc.code}: {detail[:2000]}") from exc

    def get_parts(self, eid: str) -> list[dict]:
        return self.request(f"/api/v10/parts/d/{DID}/w/{WID}/e/{eid}")  # type: ignore[return-value]

    def get_metadata(self, eid: str, part_id: str) -> dict:
        pid = urllib.parse.quote(part_id, safe="")
        return self.request(f"/api/v10/metadata/d/{DID}/w/{WID}/e/{eid}/p/{pid}")

    def set_material(self, eid: str, part_id: str, material: dict) -> dict:
        pid = urllib.parse.quote(part_id, safe="")
        body = {"properties": [{"propertyId": MATERIAL_PROPERTY_ID, "value": material}]}
        return self.request(
            f"/api/v10/metadata/d/{DID}/w/{WID}/e/{eid}/p/{pid}",
            method="POST",
            data=body,
        )


def multipart_form(fields: dict[str, object], file_field: str, file_path: Path) -> tuple[bytes, str]:
    boundary = "----codex-onshape-boundary-" + str(int(time.time() * 1000))
    parts: list[bytes] = []
    for key, value in fields.items():
        if isinstance(value, bool):
            text = "true" if value else "false"
        else:
            text = str(value)
        parts.append(
            f"--{boundary}\r\nContent-Disposition: form-data; name=\"{key}\"\r\n\r\n{text}\r\n".encode()
        )
    filename = file_path.name
    content_type = mimetypes.guess_type(filename)[0] or "application/octet-stream"
    parts.append(
        (
            f"--{boundary}\r\n"
            f"Content-Disposition: form-data; name=\"{file_field}\"; filename=\"{filename}\"\r\n"
            f"Content-Type: {content_type}\r\n\r\n"
        ).encode()
    )
    parts.append(file_path.read_bytes())
    parts.append(f"\r\n--{boundary}--\r\n".encode())
    return b"".join(parts), f"multipart/form-data; boundary={boundary}"


def get_material_template(client: OnshapeClient, template_eid: str) -> dict:
    parts = client.get_parts(template_eid)
    first = next(part for part in parts if part.get("bodyType") == "solid" and not part.get("isMesh"))
    metadata = client.get_metadata(template_eid, first["partId"])
    for prop in metadata.get("properties", []):
        if prop.get("propertyId") == MATERIAL_PROPERTY_ID and prop.get("value"):
            return prop["value"]
    raise RuntimeError(f"Could not find a material template on {template_eid}")


def upload_step(client: OnshapeClient, step_path: Path, import_name: str, events: list[dict]) -> str:
    fields = {
        "encodedFilename": f"{import_name}.step",
        "formatName": "STEP",
        "translate": True,
        "storeInDocument": True,
        "importWithinDocument": True,
        "allowFaultyParts": True,
        "joinAdjacentSurfaces": False,
        "createComposite": False,
        "flattenAssemblies": True,
        "unit": "millimeter",
        "notifyUser": False,
    }
    body, content_type = multipart_form(fields, "file", step_path)
    upload = client.request(
        f"/api/v10/blobelements/d/{DID}/w/{WID}",
        method="POST",
        data=body,
        headers={"Content-Type": content_type},
        timeout=180,
    )
    translation_id = upload.get("translationId")
    events.append(
        {
            "event": "upload",
            "status": upload.get("_status"),
            "elementId": upload.get("id"),
            "translationId": translation_id,
        }
    )
    if not translation_id:
        raise RuntimeError(f"Upload did not return a translationId: {upload}")

    translation = {}
    for _ in range(80):
        translation = client.request(f"/api/v10/translations/{translation_id}")
        state = translation.get("requestState")
        events.append({"event": "translation_state", "state": state})
        if state in {"DONE", "FAILED", "CANCELLED"}:
            break
        time.sleep(3)

    if translation.get("requestState") != "DONE":
        raise RuntimeError(f"Translation did not complete: {translation}")
    result_eids = translation.get("resultElementIds") or []
    if not result_eids:
        raise RuntimeError(f"Translation had no result elements: {translation}")
    events.append({"event": "translation_done", "partStudioElementId": result_eids[0]})
    return result_eids[0]


def create_sim_assembly(client: OnshapeClient, source_eid: str, assembly_name: str, events: list[dict]) -> str:
    assembly = client.request(
        f"/api/v10/assemblies/d/{DID}/w/{WID}",
        method="POST",
        data={"name": assembly_name},
    )
    assembly_eid = assembly.get("id")
    if not assembly_eid:
        raise RuntimeError(f"Assembly create response missing id: {assembly}")
    events.append({"event": "create_assembly", "assemblyElementId": assembly_eid})

    insert_payload = {
        "documentId": DID,
        "elementId": source_eid,
        "isWholePartStudio": True,
        "isAssembly": False,
        "includePartTypes": ["PARTS"],
    }
    insert = client.request(
        f"/api/v10/assemblies/d/{DID}/w/{WID}/e/{assembly_eid}/instances",
        method="POST",
        data=insert_payload,
    )
    events.append(
        {"event": "insert_partstudio", "keys": sorted(insert.keys()) if isinstance(insert, dict) else []}
    )
    return assembly_eid


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", type=Path, default=ROOT / ".secrets" / "onshape.env")
    parser.add_argument(
        "--step",
        type=Path,
        default=CAD_TEST / "out" / "step" / "connected_one_leg_load_path_L0.step",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=CAD_TEST / "out" / "connected_one_leg_load_path_L0_manifest.json",
    )
    parser.add_argument("--import-name", default="connected_one_leg_load_path_L0")
    parser.add_argument("--assembly-name", default="MuJoCo load simulation - L0 connected load path")
    parser.add_argument("--material-template-eid", default=PETG_TEMPLATE_EID)
    parser.add_argument(
        "--out",
        type=Path,
        default=ARTIFACT_DIR / "mujoco_sim_connected_one_leg_result.json",
    )
    args = parser.parse_args()

    load_env(args.env)
    base_url = os.environ.get("ONSHAPE_BASE_URL", "https://cad.onshape.com")
    client = OnshapeClient(base_url)

    events: list[dict] = []
    material = get_material_template(client, args.material_template_eid)
    events.append({"event": "material_template", "material": material.get("displayName")})

    partstudio_eid = upload_step(client, args.step, args.import_name, events)
    parts = client.get_parts(partstudio_eid)
    solid_parts = [part for part in parts if part.get("bodyType") == "solid" and not part.get("isMesh")]

    updated: list[dict] = []
    failed: list[dict] = []
    for part in solid_parts:
        try:
            client.set_material(partstudio_eid, part["partId"], material)
            updated.append({"name": part.get("name"), "partId": part.get("partId")})
        except Exception as exc:  # noqa: BLE001 - persist all API failures in the artifact.
            failed.append(
                {
                    "name": part.get("name"),
                    "partId": part.get("partId"),
                    "error": str(exc)[:300],
                }
            )
    events.append({"event": "material_assignment", "updated": len(updated), "failed": len(failed)})

    assembly_eid = create_sim_assembly(client, partstudio_eid, args.assembly_name, events)
    assembly = client.request(f"/api/v10/assemblies/d/{DID}/w/{WID}/e/{assembly_eid}")
    instance_count = len(assembly.get("rootAssembly", {}).get("instances", []))

    url = f"{base_url.rstrip('/')}/documents/{DID}/w/{WID}/e/{assembly_eid}"
    result = {
        "documentId": DID,
        "workspaceId": WID,
        "sourceStep": str(args.step.resolve()),
        "sourceManifest": str(args.manifest.resolve()),
        "partStudioElementId": partstudio_eid,
        "assemblyElementId": assembly_eid,
        "assemblyName": args.assembly_name,
        "solidPartCount": len(solid_parts),
        "assemblyInstanceCount": instance_count,
        "materialUpdatedCount": len(updated),
        "materialFailures": failed,
        "url": url,
        "events": events,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

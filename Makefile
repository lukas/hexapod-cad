# hexapod-cad: CAD/print/BuildViz pipeline for the STS3215 hexapod.
# Split from lukas/hexapod (hexapod_walker/prototype_sts3215) on 2026-09-15.
# Every Python target runs through `uv run` in this repo's venv (`uv sync`).

PY       := uv run python
HEXAPOD  ?= ../hexapod        # robot repo checkout, for `make mesh-sync` and the `sim` extra
BUILDVIZ_ROOT ?= $(HOME)/buildviz

.PHONY: help check-cad check-cad-fast validate render build step-export verify-prototype verify-fast verify-changed view-build verify-buildviz push-cloud regen-fasteners bom-fasteners mesh mesh-sync check-strength prepare-onshape-strength buildviz-chassis-reinforcement buildviz-tibia-yoke-reinforcement buildviz-horn-compression-limiters buildviz-premade-chorn-56 clean-artifacts

help:
	@echo "hexapod-cad targets:"
	@echo "  check-cad         Build STLs, validate against design_spec.yaml, render views, write cad_report.md."
	@echo "  check-cad-fast    Same, skipping the slow workspace-sweep collision check."
	@echo "  build             Re-run build_all.py -> step_prototype/ (STEP truth) + stl_prototype/ + stl_reference/."
	@echo "  step-export       Only the STEP-first BREP exporter + print-set install."
	@echo "  verify-prototype  Full _verify_prototype.main() suite (~95-270 s); verify-fast / verify-changed for the inner loop."
	@echo "  view-build        PyVista build inspector (colored parts, exploded-view slider)."
	@echo "  verify-buildviz   Regenerate full_robot_viz/scene.json, verify, bump a new version into the BuildViz hub."
	@echo "  push-cloud        Mirror the local default BuildViz version to the cloud hub."
	@echo "  mesh              Build the as-built MuJoCo mesh model (mesh_mujoco/assets + hexapod_mesh.xml)."
	@echo "  mesh-sync         mesh, then copy the result into \$$(HEXAPOD)/hexapod_walker/prototype_sts3215/mesh_mujoco (commit it there)."
	@echo "  check-strength    OPT-IN FEA failure-point check (Gmsh + CalculiX): make check-strength MATERIAL=petg"
	@echo "  regen-fasteners / bom-fasteners   Fastener cache STLs / inject the fastener BOM into docs/BOM.md."
	@echo "  buildviz-*        Concept test kits (see the targets)."

check-cad:
	$(PY) scripts/check_cad.py

check-cad-fast:
	$(PY) scripts/check_cad.py --fast

validate:
	$(PY) scripts/validate_geometry.py

render:
	$(PY) scripts/render_views.py

build:
	$(PY) build_all.py --skip-assembly

step-export:
	$(PY) build_step_prototype.py
	$(PY) -c "import step_pipeline; step_pipeline.install_printables()"

verify-prototype:
	$(PY) _verify_prototype.py

verify-fast:
	$(PY) _verify_prototype.py --fast

verify-changed:
	$(PY) _verify_prototype.py --changed

view-build:
	$(PY) scripts/inspect_build.py $(ARGS)

# The as-built MuJoCo mesh model. The robot repo tracks the OUTPUT of this
# target (training's default model_source is "mesh"); regenerate here after
# any geometry change and commit the copied files there together with the
# matching hexapod_core/geometry.py numbers.
mesh:
	@test -f extra_stl/hex_raised_platform_110_h28_screen.stl || $(PY) tools/make_xtool_hex_raised_platform.py
	cd mesh_mujoco && $(PY) build_mesh_model.py --no-render

mesh-sync: mesh
	rsync -a --delete mesh_mujoco/assets/ $(HEXAPOD)/hexapod_walker/prototype_sts3215/mesh_mujoco/assets/
	cp mesh_mujoco/hexapod_mesh.xml $(HEXAPOD)/hexapod_walker/prototype_sts3215/mesh_mujoco/hexapod_mesh.xml
	@echo "copied into $(HEXAPOD); review and commit there"

BUILDVIZ_BUILD_ID ?= prototype_sts3215
BUILDVIZ_KEEP     ?= 20
SWEEP             ?=
verify-buildviz:
	$(PY) tools/full_robot_viz_build.py
	$(PY) tools/buildviz_checks.py full_robot_viz/scene.json \
		--emit --publish --build-id $(BUILDVIZ_BUILD_ID) --keep $(BUILDVIZ_KEEP)
	npx buildviz validate $(BUILDVIZ_BUILD_ID) --json >/dev/null && echo "    buildviz validate: OK"
	@if [ -n "$(SWEEP)" ]; then npx buildviz sweep $(BUILDVIZ_BUILD_ID) --json >/dev/null && echo "    buildviz sweep: OK"; fi
	-$(PY) tools/push_cloud_buildviz.py --build-id $(BUILDVIZ_BUILD_ID) || \
		echo "    WARNING: cloud mirror failed (offline / no BUILDVIZ_API_KEY?); run 'make push-cloud' later"

push-cloud:
	$(PY) tools/push_cloud_buildviz.py --build-id $(BUILDVIZ_BUILD_ID)

MATERIAL ?= petg
PARTS    ?=
SOLVER   ?= both
STRENGTH_ARGS := --material $(MATERIAL) --solver $(SOLVER)
ifneq ($(strip $(PARTS)),)
STRENGTH_ARGS += --parts $(PARTS)
endif
check-strength:
	$(PY) strength/__main__.py $(STRENGTH_ARGS)

ONSHAPE_STUDY_ARGS ?=
prepare-onshape-strength:
	$(PY) strength/prepare_onshape_study.py $(ONSHAPE_STUDY_ARGS)

buildviz-chassis-reinforcement:
	$(PY) concepts/chassis_reinforcement/make_chassis_reinforcement_concept.py
	npx buildviz push --build-id prototype_sts3215/chassis-reinforcement-test \
		--scene concepts/chassis_reinforcement/scene.json \
		--design-spec concepts/chassis_reinforcement/design_spec.yaml \
		--name "STS3215 chassis reinforcement test kit" --bump --upload-assets \
		--message "regenerate chassis/yaw-pocket reinforcement concept"

buildviz-tibia-yoke-reinforcement:
	$(PY) concepts/tibia_yoke_reinforcement/make_tibia_yoke_reinforcement_concept.py
	npx buildviz push --build-id prototype_sts3215/tibia-yoke-reinforcement-test \
		--scene concepts/tibia_yoke_reinforcement/scene.json \
		--design-spec concepts/tibia_yoke_reinforcement/design_spec.yaml \
		--name "STS3215 tibia/yoke reinforcement load-path test" --bump --upload-assets \
		--message "focused tibia/yoke load-path reinforcement concept"

buildviz-horn-compression-limiters:
	$(PY) concepts/horn_compression_limiters/make_horn_compression_limiter_concept.py
	$(PY) concepts/horn_compression_limiters/verify_workspace_variant.py
	$(PY) tools/buildviz_checks.py concepts/horn_compression_limiters/scene.json --pitch 0.4 --emit --json
	npx buildviz push --build-id prototype_sts3215/horn-compression-limiters \
		--scene concepts/horn_compression_limiters/scene.json \
		--design-spec concepts/horn_compression_limiters/design_spec.yaml \
		--name "STS3215 horn compression-limiter experiment" --bump --upload-assets \
		--message "load spacer dimensions from editable config; retain coupon-tested bore"
	-$(PY) tools/push_cloud_buildviz.py --build-id prototype_sts3215/horn-compression-limiters

buildviz-premade-chorn-56:
	$(PY) concepts/premade_chorn_56/build_premade_chorn_hardware_step.py
	$(PY) concepts/premade_chorn_56/make_premade_chorn_56_variant.py
	npx buildviz check concepts/premade_chorn_56 \
		--checks mesh_overlap,declared_interference,scene_meta,disconnected_components --max-pairs 1000 --emit
	jq -e '.summary.fail == 0' concepts/premade_chorn_56/buildviz_checks.json >/dev/null
	npx buildviz push --build-id prototype_sts3215/premade-chorn-56 \
		--scene concepts/premade_chorn_56/scene.json \
		--design-spec concepts/premade_chorn_56/design_spec.yaml \
		--name "STS3215 premade 56 mm C-horn experiment" --bump --upload-assets \
		--message "reinforce tibia root and upper-bearing load path; add rear carrier fastener"
	-$(PY) tools/push_cloud_buildviz.py --build-id prototype_sts3215/premade-chorn-56

regen-fasteners:
	$(PY) scripts/make_fastener_meshes.py

bom-fasteners:
	$(PY) scripts/render_fastener_bom.py --inject $(CURDIR)/docs/BOM.md

clean-artifacts:
	rm -rf artifacts

PY ?= python

.PHONY: help build check check-scene audit report view-buildviz

help:
	@echo "AK40 hexapod prototype targets:"
	@echo "  make build         - regenerate stl_prototype + stl_reference + design_budget.md"
	@echo "  make check         - build + watertightness/volume verification"
	@echo "  make check-scene   - full-assembly overlap sweep over scene.json (all 200 instances)"
	@echo "  make audit         - fastener tool-access + ROM/combined-pose audits (~2 min)"
	@echo "  make report        - print the mass/torque budget table (no STL writes)"
	@echo "  make view-buildviz - regenerate + register the BuildViz scene, open the hub"

build:
	$(PY) hexapod_ak40.py

check:
	$(PY) hexapod_ak40.py --check

report:
	$(PY) hexapod_ak40.py --report-only

check-scene:
	$(PY) full_robot_viz/make_scene.py
	$(PY) scripts/check_scene.py

audit:
	$(PY) scripts/access_audit.py
	$(PY) scripts/rom_audit.py

view-buildviz:
	cp design_spec.yaml full_robot_viz/design_spec.yaml
	$(PY) full_robot_viz/make_scene.py
	npx buildviz hub --detach
	npx buildviz register full_robot_viz --build-id prototype_ak40
	open "http://127.0.0.1:5183/?build=prototype_ak40"

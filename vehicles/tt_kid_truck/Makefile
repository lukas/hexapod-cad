UV_CAD = uv run --no-project --python 3.12 --with build123d --with trimesh --with numpy
BUILDVIZ ?= npx buildviz
PROJECT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
REPO_ROOT := $(abspath $(PROJECT_DIR)/../..)
BUILD_DIR := $(PROJECT_DIR)/out
CLOUD_MIRROR := $(REPO_ROOT)/tools/push_cloud_buildviz.py

.PHONY: build check view-buildviz push-cloud pack

build:
	$(UV_CAD) python "$(PROJECT_DIR)/build_tt_truck.py"

check: build
	$(UV_CAD) python "$(PROJECT_DIR)/build_tt_truck.py" --check-only
	$(BUILDVIZ) compat "$(BUILD_DIR)" --json
	$(BUILDVIZ) validate "$(BUILD_DIR)" --json
	$(BUILDVIZ) check "$(BUILD_DIR)" --emit --json

view-buildviz: check
	$(BUILDVIZ) push --project tt-kid-truck --build chassis --version main --no-snapshot --scene "$(BUILD_DIR)/scene.json" --upload-assets -m "add removable classic pickup cab, hood, open bed, wheel arches, grille, and lights"
	-$(MAKE) push-cloud

push-cloud:
	-uv run --no-project python "$(CLOUD_MIRROR)" --build-id tt-kid-truck/chassis

pack: check
	$(BUILDVIZ) pack "$(BUILD_DIR)" --assembly printable --printer x1c --export --out "$(BUILD_DIR)/plates"

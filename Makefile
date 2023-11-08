# Copyright (c) 2021-2023 by Phase Advanced Sensor Systems Corp.
MODULE      := xtalx
MODULE_VERS := 1.0.8
MODULES := \
	xtalx/i2c_bridge/*.py \
	xtalx/p_sensor/*.py \
	xtalx/z_sensor/*.py \
	xtalx/tools/config/*.py \
	xtalx/tools/i2c/*.py \
	xtalx/tools/influxdb/*.py \
	xtalx/tools/math/*.py \
	xtalx/tools/p_sensor/*.py \
	xtalx/tools/usb/*.py \
	xtalx/tools/z_sensor/*.py

FLAKE_MODULES := xtalx
LINT_MODULES  := xtalx
WHEEL_PATH    := dist/$(MODULE)-$(MODULE_VERS)-py3-none-any.whl
TGZ_PATH      := dist/$(MODULE)-$(MODULE_VERS).tar.gz

.PHONY: all
all: test packages

.PHONY: clean
clean:
	rm -rf dist $(MODULE).egg-info build
	find . -name "*.pyc" | xargs rm 2>/dev/null || true
	find . -name __pycache__ | xargs rm -r 2>/dev/null || true

.PHONY: test
test: flake8 lint

.PHONY: flake8
flake8:
	python3 -m flake8 $(FLAKE_MODULES)

.PHONY: lint
lint:
	pylint -j2 $(LINT_MODULES)

.PHONY: install
install: $(WHEEL_PATH) | uninstall
	sudo python3 -m pip install $(WHEEL_PATH)

.PHONY: uninstall
uninstall:
	sudo python3 -m pip uninstall -y $(MODULE)

.PHONY: packages
packages: $(WHEEL_PATH)

.PHONY: publish
publish: all
	python3 -m twine upload $(WHEEL_PATH) $(TGZ_PATH)

$(WHEEL_PATH): setup.py setup.cfg $(MODULES)
	python3 setup.py --quiet sdist bdist_wheel
	python3 -m twine check $@

COMPOSE ?= docker compose --env-file docker/.env -f docker/compose.yaml
DOCS_PYTHON ?= docs/.venv/bin/python
DOCS_OUTPUT ?= docs/site

.DEFAULT_GOAL := help
.PHONY: help build run attach docs docs-build

help:
	@printf '%s\n' \
	  'Simulador SCI: ejecutar los comandos desde la carpeta del proyecto.' \
	  '' \
	  '  make build     Construir la imagen del simulador.' \
	  '  make run       Ejecutar en esta terminal; Ctrl+C detiene y retira el contenedor.' \
	  '  make help      Mostrar esta ayuda.'

build:
	$(COMPOSE) build

attach:
	$(COMPOSE) exec simulator bash
	
run:
	@running_simulator="$$($(COMPOSE) ps --status running --quiet simulator)" || exit $$?; \
	  if [ -n "$$running_simulator" ]; then \
	    printf '%s\n' 'La simulación ya está en ejecución; usa make attach para abrir otra terminal.' >&2; \
	    exit 1; \
	  fi; \
	  trap '$(COMPOSE) down' EXIT; \
	  trap 'exit 130' INT; \
	  trap 'exit 143' TERM; \
	  $(COMPOSE) up --no-build --pull never --abort-on-container-exit --exit-code-from simulator

docs:
	$(DOCS_PYTHON) -m mkdocs serve --config-file docs/mkdocs.yml --dev-addr 127.0.0.1:8000

docs-build:
	$(DOCS_PYTHON) -m mkdocs build --strict --config-file docs/mkdocs.yml --site-dir "$(abspath $(DOCS_OUTPUT))"

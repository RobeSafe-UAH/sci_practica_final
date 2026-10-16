COMPOSE ?= docker compose
DOCS_PYTHON ?= .venv-docs/bin/python
DOCS_OUTPUT ?= site

.DEFAULT_GOAL := help
.PHONY: help build run docs docs-build

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
	@trap '$(COMPOSE) down' EXIT; \
	  trap 'exit 130' INT; \
	  trap 'exit 143' TERM; \
	  $(COMPOSE) up --no-build --pull never --abort-on-container-exit --exit-code-from simulator

docs:
	$(DOCS_PYTHON) -m mkdocs serve --dev-addr 127.0.0.1:8000

docs-build:
	$(DOCS_PYTHON) -m mkdocs build --strict --site-dir $(DOCS_OUTPUT)

COMPOSE ?= docker compose
DOCS_PYTHON ?= .venv-docs/bin/python
DOCS_OUTPUT ?= site

.DEFAULT_GOAL := help
.PHONY: help build up run restart recreate down logs status docs docs-build

help:
	@printf '%s\n' \
	  'Simulador SCI: ejecutar los comandos desde la carpeta del proyecto.' \
	  '' \
	  '  make build     Construir la imagen del simulador.' \
	  '  make run       Arrancar la imagen construida y esperar a que esté saludable.' \
	  '  make up        Alias de make run.' \
	  '  make restart   Reiniciar el simulador tras editar config/scenario.yaml.' \
	  '  make recreate  Recrear el contenedor tras editar .env.' \
	  '  make down      Detener y retirar el contenedor.' \
	  '  make logs      Seguir los registros; Ctrl+C deja de seguirlos.' \
	  '  make status    Consultar el estado del contenedor.' \
	  '  make help      Mostrar esta ayuda.' \
	  '' \
	  'Documentación (requiere requirements-docs.txt en .venv-docs):' \
	  '  make docs        Servir la documentación en http://127.0.0.1:8000.' \
	  '  make docs-build  Comprobar y generar el sitio en site/.'

build:
	$(COMPOSE) build

up:
	$(COMPOSE) up -d --no-build --pull never --wait

run: up

restart:
	$(COMPOSE) restart simulator

recreate:
	$(COMPOSE) up -d --force-recreate --no-build --pull never --wait

down:
	$(COMPOSE) down

logs:
	$(COMPOSE) logs -f simulator

status:
	$(COMPOSE) ps

docs:
	$(DOCS_PYTHON) -m mkdocs serve --dev-addr 127.0.0.1:8000

docs-build:
	$(DOCS_PYTHON) -m mkdocs build --strict --site-dir $(DOCS_OUTPUT)
